#include "decision.hpp"
#include "arm.hpp"
#include "config.hpp"

#include <chrono>
#include <array>
#include <cmath>
#include <cstdio>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace {

constexpr auto DECISION_PERIOD = std::chrono::microseconds(1'000'000 / DECISION_LOOP_HZ);
constexpr auto STABLE_DART_TIME = 2s;
constexpr float HOMING_MOVE_DURATION = 4.0f;
constexpr float MIN_SEGMENT_DURATION = 2.0f;
constexpr float MAX_SEGMENT_DURATION = 9.0f;
constexpr float JOINT_CRUISE_SPEED_RAD_S = 0.45f;
constexpr float FLYTHROUGH_JOINT_THRESHOLD = 0.12f;
constexpr auto MOVE_TIMEOUT = 15s;
constexpr auto GRIPPER_SETTLE = 1s;
constexpr auto WAYPOINT_SETTLE = 300ms;
constexpr auto RESET_COOLDOWN = 2s;
constexpr const char* SEARCH_WAIT_POSE = "backward_prep";

const std::vector<const char*> SEARCH_READY_SEQUENCE = {
    "home",
    "forward_prep",
    SEARCH_WAIT_POSE,
};

// ── Detection zones ──────────────────────────────────────────────────────────
// Tune these centers and half-sizes after camera calibration.  Units are
// metres in the VisionModule world frame.  The defaults below are seeded from
// the clusters seen during manual camera testing.
constexpr float DEFAULT_ZONE_HALF_W_M = 0.020f;
constexpr float DEFAULT_ZONE_HALF_H_M = 0.020f;

enum class TrajectoryId {
    Slot0,
    Slot1,
    Slot2,
    Slot3,
};

struct DartZone {
    const char*  name;
    float        cx;
    float        cy;
    float        half_w;
    float        half_h;
    TrajectoryId trajectory;
};

const std::vector<DartZone> DART_ZONES = {
    // name       center_x  center_y  half_width              half_height              trajectory
    {"slot_1",   -0.062f,    0.029f,  DEFAULT_ZONE_HALF_W_M,  DEFAULT_ZONE_HALF_H_M,  TrajectoryId::Slot1},
    {"slot_2",    0.068f,    0.026f,  DEFAULT_ZONE_HALF_W_M,  DEFAULT_ZONE_HALF_H_M,  TrajectoryId::Slot2},
    {"slot_3",    0.105f,   -0.008f,  DEFAULT_ZONE_HALF_W_M,  DEFAULT_ZONE_HALF_H_M,  TrajectoryId::Slot3},
};

bool contains(const DartZone& zone, const Detection& det)
{
    return std::abs(det.x - zone.cx) <= zone.half_w &&
           std::abs(det.y - zone.cy) <= zone.half_h;
}

int find_zone(const Detection& det)
{
    for (size_t i = 0; i < DART_ZONES.size(); ++i) {
        if (contains(DART_ZONES[i], det))
            return static_cast<int>(i);
    }
    return -1;
}

// ── Trajectories ─────────────────────────────────────────────────────────────
// Edit these tables when a fixed dart slot needs a different pickup sequence.
// Gripper actions are tied to the step they should run after arriving at.
enum class GripperAction {
    None,
    Open,
    Close,
};

struct TrajectoryStep {
    const char*   position;
    GripperAction gripper;
};

const std::vector<TrajectoryStep> TRAJECTORY_SLOT_0 = {
    {"s0_prep",        GripperAction::Open},
    {"s0_grasp",       GripperAction::Close},
    {"s0_pulled_out",  GripperAction::None},
    {"backward_prep",  GripperAction::None},
    {"forward_prep",   GripperAction::None},
    {"loading",        GripperAction::None},
    {"home",           GripperAction::None},
};

const std::vector<TrajectoryStep> TRAJECTORY_SLOT_1 = {
    {"s1_prep",        GripperAction::Open},
    {"s1_grasp",       GripperAction::Close},
    {"s1_pulled_out",  GripperAction::None},
    {"backward_prep",  GripperAction::None},
    {"forward_prep",   GripperAction::None},
    {"loading",        GripperAction::None},
    {"home",           GripperAction::None},
};

const std::vector<TrajectoryStep> TRAJECTORY_SLOT_2 = {
    {"s2_prep",        GripperAction::Open},
    {"s2_grasp",       GripperAction::Close},
    {"s2_pulled_out",  GripperAction::None},
    {"backward_prep",  GripperAction::None},
    {"forward_prep",   GripperAction::None},
    {"loading",        GripperAction::None},
    {"home",           GripperAction::None},
};

const std::vector<TrajectoryStep> TRAJECTORY_SLOT_3 = {
    {"s3_prep",        GripperAction::Open},
    {"s3_grasp",       GripperAction::Close},
    {"s3_pulled_out",  GripperAction::None},
    {"backward_prep",  GripperAction::None},
    {"forward_prep",   GripperAction::None},
    {"loading",        GripperAction::None},
    {"home",           GripperAction::None},
};

const char* trajectory_name(TrajectoryId id)
{
    switch (id) {
    case TrajectoryId::Slot0: return "trajectory_slot_0";
    case TrajectoryId::Slot1: return "trajectory_slot_1";
    case TrajectoryId::Slot2: return "trajectory_slot_2";
    case TrajectoryId::Slot3: return "trajectory_slot_3";
    default:                  return "?";
    }
}

const std::vector<TrajectoryStep>& trajectory_steps(TrajectoryId id)
{
    switch (id) {
    case TrajectoryId::Slot0: return TRAJECTORY_SLOT_0;
    case TrajectoryId::Slot1: return TRAJECTORY_SLOT_1;
    case TrajectoryId::Slot2: return TRAJECTORY_SLOT_2;
    case TrajectoryId::Slot3: return TRAJECTORY_SLOT_3;
    default:                  return TRAJECTORY_SLOT_0;
    }
}

float clampf(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

float max_joint_delta(const std::array<float, 4>& from,
                      const std::array<float, 4>& to)
{
    float max_delta = 0.f;
    for (int i = 0; i < 4; ++i)
        max_delta = std::max(max_delta, std::abs(to[i] - from[i]));
    return max_delta;
}

float duration_for_segment(const std::array<float, 4>& from,
                           const std::array<float, 4>& to)
{
    const float max_delta = max_joint_delta(from, to);
    const float duration = max_delta / JOINT_CRUISE_SPEED_RAD_S;
    return clampf(duration, MIN_SEGMENT_DURATION, MAX_SEGMENT_DURATION);
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────

DecisionModule::DecisionModule(TripleBuffer<Detection>& detection_buf, ArmModule& arm)
    : detection_buf_(detection_buf)
    , arm_(arm)
{}

DecisionModule::~DecisionModule() { stop(); }

bool DecisionModule::start() {
    running_ = true;
    state_   = State::Idle;
    thread_  = std::thread(&DecisionModule::loop, this);
    return true;
}

void DecisionModule::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

bool DecisionModule::sleep_interruptible(std::chrono::milliseconds duration) const
{
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (running_ && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(20ms);
    }
    return running_;
}

bool DecisionModule::wait_for_goal(std::chrono::seconds timeout) const
{
    using Clock = std::chrono::steady_clock;
    const auto deadline = Clock::now() + timeout;
    ArmState state{};

    while (running_ && Clock::now() < deadline) {
        if (!arm_.read_state(state)) {
            std::this_thread::sleep_for(20ms);
            continue;
        }
        if (!state.hw_ok) {
            std::fprintf(stderr, "[Decision] Arm hardware went offline while moving\n");
            return false;
        }
        if (state.reached_goal) return true;
        std::this_thread::sleep_for(50ms);
    }

    return false;
}

bool DecisionModule::wait_until_near(const std::array<float, 4>& target,
                                     float threshold,
                                     std::chrono::seconds timeout) const
{
    using Clock = std::chrono::steady_clock;
    const auto deadline = Clock::now() + timeout;
    ArmState state{};

    while (running_ && Clock::now() < deadline) {
        if (!arm_.read_state(state)) {
            std::this_thread::sleep_for(20ms);
            continue;
        }
        if (!state.hw_ok) {
            std::fprintf(stderr, "[Decision] Arm hardware went offline while moving\n");
            return false;
        }
        if (max_joint_delta(state.joints, target) <= threshold)
            return true;
        std::this_thread::sleep_for(40ms);
    }

    return false;
}

bool DecisionModule::move_to_and_wait(const std::string& name,
                                      float fallback_duration,
                                      bool stop_at_goal)
{
    ArmState state{};
    std::array<float, 4> target{};

    if (!arm_.get_position(name, target)) {
        std::fprintf(stderr, "[Decision] Unknown or unavailable position: %s\n", name.c_str());
        return false;
    }

    const bool have_state = arm_.read_state(state);
    const float duration = have_state
        ? duration_for_segment(state.joints, target)
        : fallback_duration;

    std::printf("[Decision] Move -> %s  duration=%.2fs%s\n",
                name.c_str(), duration, stop_at_goal ? "" : "  fly-through");
    arm_.move_joints(target, duration);

    // Give the arm control loop time to consume the command before accepting
    // reached_goal. This still allows a no-op move, such as home->home, to pass.
    if (!sleep_interruptible(std::chrono::milliseconds((1000 / ARM_LOOP_HZ) * 3)))
        return false;

    if (stop_at_goal) {
        if (!wait_for_goal(MOVE_TIMEOUT)) {
            std::fprintf(stderr, "[Decision] Timed out moving to %s\n", name.c_str());
            return false;
        }

        if (!sleep_interruptible(std::chrono::duration_cast<std::chrono::milliseconds>(WAYPOINT_SETTLE)))
            return false;
    } else {
        if (!wait_until_near(target, FLYTHROUGH_JOINT_THRESHOLD, MOVE_TIMEOUT)) {
            std::fprintf(stderr, "[Decision] Timed out approaching fly-through waypoint %s\n",
                         name.c_str());
            return false;
        }
    }

    return true;
}

bool DecisionModule::move_to_search_pose()
{
    std::printf("[Decision] Preparing search pose: home -> forward_prep -> %s\n",
                SEARCH_WAIT_POSE);

    for (size_t i = 0; i < SEARCH_READY_SEQUENCE.size(); ++i) {
        const bool last_step = (i + 1 == SEARCH_READY_SEQUENCE.size());
        if (!move_to_and_wait(SEARCH_READY_SEQUENCE[i], HOMING_MOVE_DURATION, last_step))
            return false;
    }

    std::printf("[Decision] Search waiting at %s\n", SEARCH_WAIT_POSE);
    return true;
}

bool DecisionModule::execute_pickup_trajectory(int zone_index)
{
    if (zone_index < 0 || zone_index >= static_cast<int>(DART_ZONES.size())) {
        std::fprintf(stderr, "[Decision] Invalid zone index: %d\n", zone_index);
        return false;
    }

    const DartZone& zone = DART_ZONES[zone_index];
    const auto& steps = trajectory_steps(zone.trajectory);

    std::printf("[Decision] Starting %s for %s; camera updates ignored until reset\n",
                trajectory_name(zone.trajectory), zone.name);

    for (size_t i = 0; i < steps.size(); ++i) {
        const TrajectoryStep& step = steps[i];
        if (!running_) return false;

        const bool last_step = (i + 1 == steps.size());
        const bool stop_at_goal = last_step || step.gripper != GripperAction::None;

        if (!move_to_and_wait(step.position, HOMING_MOVE_DURATION, stop_at_goal))
            return false;

        if (step.gripper == GripperAction::Open) {
            std::printf("[Decision] Open gripper at %s\n", step.position);
            arm_.release();
            if (!sleep_interruptible(std::chrono::duration_cast<std::chrono::milliseconds>(GRIPPER_SETTLE)))
                return false;
        } else if (step.gripper == GripperAction::Close) {
            std::printf("[Decision] Close gripper at %s\n", step.position);
            arm_.grip();
            if (!sleep_interruptible(std::chrono::duration_cast<std::chrono::milliseconds>(GRIPPER_SETTLE)))
                return false;
        }
    }

    std::printf("[Decision] Pickup trajectory complete; arm is back at home\n");
    return true;
}

void DecisionModule::reset_stability()
{
    stable_zone_index_ = -1;
}

bool DecisionModule::update_stability(const Detection& det,
                                      std::chrono::steady_clock::time_point now)
{
    if (!det.valid) {
        if (stable_zone_index_ >= 0)
            std::printf("[Decision] Lost detection; stability reset\n");
        reset_stability();
        return false;
    }

    const int zone_index = find_zone(det);
    if (zone_index < 0) {
        if (stable_zone_index_ >= 0)
            std::printf("[Decision] Dart left all zones; stability reset\n");
        reset_stability();
        return false;
    }

    const DartZone& zone = DART_ZONES[zone_index];

    if (stable_zone_index_ != zone_index) {
        stable_zone_index_ = zone_index;
        stable_t0_ = now;
        std::printf("[Decision] Dart entered %s bbox: det=(%.3f, %.3f), center=(%.3f, %.3f), half=(%.3f, %.3f); waiting %.1fs\n",
                    zone.name, det.x, det.y, zone.cx, zone.cy, zone.half_w, zone.half_h,
                    std::chrono::duration<float>(STABLE_DART_TIME).count());
        return false;
    }

    if (now - stable_t0_ >= STABLE_DART_TIME) {
        triggered_zone_index_ = zone_index;
        std::printf("[Decision] Dart stayed in %s for %.1fs; triggering %s\n",
                    zone.name,
                    std::chrono::duration<float>(now - stable_t0_).count(),
                    trajectory_name(zone.trajectory));
        return true;
    }

    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// State machine
// ─────────────────────────────────────────────────────────────────────────────

void DecisionModule::loop() {
    using Clock = std::chrono::steady_clock;

    Detection det{};
    ArmState  arm_state{};
    State     prev_state = State::Idle;

    state_ = State::Homing;

    while (running_) {
        const auto t0 = Clock::now();

        const bool got_arm_state = arm_.read_state(arm_state);

        if (state_ != prev_state) {
            std::printf("[Decision] -> %s\n", state_name(state_));
            prev_state = state_;
        }

        switch (state_) {
        case State::Idle:
            break;

        case State::Homing:
            if (!got_arm_state) {
                break;
            }
            if (!arm_state.hw_ok) {
                std::printf("[Decision] Waiting for arm hardware...\n");
                break;
            }
            if (move_to_search_pose()) {
                reset_stability();
                state_ = State::Searching;
            } else {
                state_ = State::Idle;
            }
            break;

        case State::Searching:
            if (!got_arm_state) {
                break;
            }
            if (!arm_state.hw_ok) {
                std::fprintf(stderr, "[Decision] Arm hardware unavailable; returning to idle\n");
                reset_stability();
                state_ = State::Idle;
                break;
            }

            if (detection_buf_.read(det) && update_stability(det, Clock::now())) {
                reset_stability();
                state_ = State::ExecutingTrajectory;
            }
            break;

        case State::ExecutingTrajectory:
            if (execute_pickup_trajectory(triggered_zone_index_)) {
                state_ = State::ResetCooldown;
            } else if (running_) {
                std::fprintf(stderr, "[Decision] Pickup trajectory failed; returning home before idle\n");
                move_to_and_wait("home", HOMING_MOVE_DURATION);
                state_ = State::Idle;
            }
            break;

        case State::ResetCooldown:
            std::printf("[Decision] Reset cooldown %.1fs before searching again\n",
                        std::chrono::duration<float>(RESET_COOLDOWN).count());
            reset_stability();
            sleep_interruptible(std::chrono::duration_cast<std::chrono::milliseconds>(RESET_COOLDOWN));
            if (running_)
                state_ = State::Homing;
            break;
        }

        std::this_thread::sleep_until(t0 + DECISION_PERIOD);
    }
}

const char* DecisionModule::state_name(State s) const {
    switch (s) {
    case State::Idle:                 return "IDLE";
    case State::Homing:               return "HOMING";
    case State::Searching:            return "SEARCHING";
    case State::ExecutingTrajectory:  return "EXECUTING_TRAJECTORY";
    case State::ResetCooldown:        return "RESET_COOLDOWN";
    default:                          return "?";
    }
}
