#include "decision.hpp"
#include "arm.hpp"
#include "config.hpp"

#include <chrono>
#include <array>
#include <cmath>
#include <cstdio>
#include <thread>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace {

constexpr auto DECISION_PERIOD = std::chrono::microseconds(1'000'000 / DECISION_LOOP_HZ);

std::chrono::milliseconds seconds_to_ms(float seconds)
{
    if (seconds < 0.f)
        seconds = 0.f;
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<float>(seconds));
}

std::chrono::milliseconds millis_to_ms(float milliseconds)
{
    if (milliseconds < 0.f)
        milliseconds = 0.f;
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<float, std::milli>(milliseconds));
}

bool contains(const DecisionZone& zone, const Detection& det)
{
    return std::abs(det.x - zone.cx) <= zone.half_w &&
           std::abs(det.y - zone.cy) <= zone.half_h;
}

int find_zone(const Detection& det, const std::vector<DecisionZone>& zones)
{
    for (size_t i = 0; i < zones.size(); ++i) {
        if (contains(zones[i], det))
            return static_cast<int>(i);
    }
    return -1;
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
                           const std::array<float, 4>& to,
                           const DecisionRuntimeConfig& config)
{
    const float max_delta = max_joint_delta(from, to);
    const float cruise_speed = (config.joint_cruise_speed_rad_s > 0.01f)
        ? config.joint_cruise_speed_rad_s
        : 0.01f;
    const float duration = max_delta / cruise_speed;
    return clampf(duration, config.min_segment_duration_s, config.max_segment_duration_s);
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────

DecisionModule::DecisionModule(TripleBuffer<Detection>& detection_buf,
                               ArmModule& arm,
                               DecisionRuntimeConfig config)
    : detection_buf_(detection_buf)
    , arm_(arm)
    , config_(std::move(config))
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

bool DecisionModule::wait_for_goal(std::chrono::milliseconds timeout) const
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
                                     std::chrono::milliseconds timeout) const
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
        ? duration_for_segment(state.joints, target, config_)
        : fallback_duration;

    std::printf("[Decision] Move -> %s  duration=%.2fs%s\n",
                name.c_str(), duration, stop_at_goal ? "" : "  fly-through");
    arm_.move_joints(target, duration);

    // Give the arm control loop time to consume the command before accepting
    // reached_goal. This still allows a no-op move, such as home->home, to pass.
    if (!sleep_interruptible(std::chrono::milliseconds((1000 / ARM_LOOP_HZ) * 3)))
        return false;

    if (stop_at_goal) {
        if (!wait_for_goal(seconds_to_ms(config_.move_timeout_s))) {
            std::fprintf(stderr, "[Decision] Timed out moving to %s\n", name.c_str());
            return false;
        }

        if (!sleep_interruptible(millis_to_ms(config_.waypoint_settle_ms)))
            return false;
    } else {
        if (!wait_until_near(target,
                             config_.flythrough_joint_threshold_rad,
                             seconds_to_ms(config_.move_timeout_s))) {
            std::fprintf(stderr, "[Decision] Timed out approaching fly-through waypoint %s\n",
                         name.c_str());
            return false;
        }
    }

    return true;
}

bool DecisionModule::move_to_search_pose()
{
    if (config_.search_ready_sequence.empty()) {
        std::fprintf(stderr, "[Decision] search_ready_sequence is empty\n");
        return false;
    }

    std::printf("[Decision] Preparing search pose");
    for (const auto& pose : config_.search_ready_sequence)
        std::printf(" -> %s", pose.c_str());
    std::printf("\n");

    for (size_t i = 0; i < config_.search_ready_sequence.size(); ++i) {
        const bool last_step = (i + 1 == config_.search_ready_sequence.size());
        if (!move_to_and_wait(config_.search_ready_sequence[i],
                              config_.homing_move_duration_s,
                              last_step))
            return false;
    }

    std::printf("[Decision] Search waiting at %s\n", config_.search_wait_pose.c_str());
    return true;
}

bool DecisionModule::execute_pickup_trajectory(int zone_index)
{
    if (zone_index < 0 || zone_index >= static_cast<int>(config_.zones.size())) {
        std::fprintf(stderr, "[Decision] Invalid zone index: %d\n", zone_index);
        return false;
    }

    const DecisionZone& zone = config_.zones[zone_index];
    auto trajectory_it = config_.trajectories.find(zone.trajectory);
    if (trajectory_it == config_.trajectories.end()) {
        std::fprintf(stderr, "[Decision] Missing trajectory '%s' for zone '%s'\n",
                     zone.trajectory.c_str(), zone.name.c_str());
        return false;
    }
    const auto& steps = trajectory_it->second;

    std::printf("[Decision] Starting %s for %s; camera updates ignored until reset\n",
                zone.trajectory.c_str(), zone.name.c_str());

    for (size_t i = 0; i < steps.size(); ++i) {
        const DecisionTrajectoryStep& step = steps[i];
        if (!running_) return false;

        const bool last_step = (i + 1 == steps.size());
        const bool stop_at_goal = last_step || step.gripper != GripperAction::None;

        if (!move_to_and_wait(step.position, config_.homing_move_duration_s, stop_at_goal))
            return false;

        if (step.gripper == GripperAction::Open) {
            std::printf("[Decision] Open gripper at %s\n", step.position.c_str());
            arm_.release();
            if (!sleep_interruptible(seconds_to_ms(config_.gripper_settle_s)))
                return false;
        } else if (step.gripper == GripperAction::Close) {
            std::printf("[Decision] Close gripper at %s\n", step.position.c_str());
            arm_.grip();
            if (!sleep_interruptible(seconds_to_ms(config_.gripper_settle_s)))
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

    const int zone_index = find_zone(det, config_.zones);
    if (zone_index < 0) {
        if (stable_zone_index_ >= 0)
            std::printf("[Decision] Dart left all zones; stability reset\n");
        reset_stability();
        return false;
    }

    const DecisionZone& zone = config_.zones[zone_index];

    if (stable_zone_index_ != zone_index) {
        stable_zone_index_ = zone_index;
        stable_t0_ = now;
        std::printf("[Decision] Dart entered %s bbox: det=(%.3f, %.3f), center=(%.3f, %.3f), half=(%.3f, %.3f); waiting %.1fs\n",
                    zone.name.c_str(), det.x, det.y, zone.cx, zone.cy, zone.half_w, zone.half_h,
                    config_.stable_dart_time_s);
        return false;
    }

    if (now - stable_t0_ >= seconds_to_ms(config_.stable_dart_time_s)) {
        triggered_zone_index_ = zone_index;
        std::printf("[Decision] Dart stayed in %s for %.1fs; triggering %s\n",
                    zone.name.c_str(),
                    std::chrono::duration<float>(now - stable_t0_).count(),
                    zone.trajectory.c_str());
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
                move_to_and_wait("home", config_.homing_move_duration_s);
                state_ = State::Idle;
            }
            break;

        case State::ResetCooldown:
            std::printf("[Decision] Reset cooldown %.1fs before searching again\n",
                        config_.reset_cooldown_s);
            reset_stability();
            sleep_interruptible(seconds_to_ms(config_.reset_cooldown_s));
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
