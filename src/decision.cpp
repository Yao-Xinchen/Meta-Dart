#include "decision.hpp"
#include "arm.hpp"
#include "config.hpp"
#include "log.hpp"

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
constexpr auto SERVO_MOVE_PERIOD = 20ms;
constexpr float SERVO_LOOKAHEAD_S = 0.35f;
constexpr float MIN_SERVO_ADVANCE_RAD = 0.04f;
constexpr float MAX_SERVO_ADVANCE_RAD = 0.18f;

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

bool contains(const DecisionZone& zone, const Detection& det, float margin = 0.f)
{
    return std::abs(det.x - zone.cx) <= zone.half_w + margin &&
           std::abs(det.y - zone.cy) <= zone.half_h + margin;
}

bool contains_pickup_guard(const DecisionZone& zone,
                           const Detection& det,
                           float default_margin)
{
    const float half_w = (zone.pickup_half_w > 0.f)
        ? zone.pickup_half_w
        : zone.half_w + default_margin;
    const float half_h = (zone.pickup_half_h > 0.f)
        ? zone.pickup_half_h
        : zone.half_h + default_margin;

    return std::abs(det.x - zone.cx) <= half_w &&
           std::abs(det.y - zone.cy) <= half_h;
}

int find_zone(const Detection& det, const std::vector<DecisionZone>& zones, float margin = 0.f)
{
    for (size_t i = 0; i < zones.size(); ++i) {
        if (contains(zones[i], det, margin))
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

std::array<float, 4> step_toward(const std::array<float, 4>& current,
                                 const std::array<float, 4>& target,
                                 float max_step)
{
    std::array<float, 4> next{};
    const float max_delta = max_joint_delta(current, target);
    const float scale = (max_delta > max_step && max_delta > 1e-6f)
        ? (max_step / max_delta)
        : 1.f;

    for (int i = 0; i < 4; ++i)
        next[i] = current[i] + (target[i] - current[i]) * scale;
    return next;
}

bool is_prep_position(const std::string& position)
{
    const std::string suffix = "_prep";
    return position.size() >= suffix.size() &&
           position.compare(position.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool is_loading_release_step(const DecisionTrajectoryStep& step)
{
    return step.position == "loading" && step.gripper == GripperAction::Open;
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

DecisionModule::MoveResult DecisionModule::wait_for_goal(std::chrono::milliseconds timeout,
                                                         int monitor_zone_index)
{
    using Clock = std::chrono::steady_clock;
    const auto deadline = Clock::now() + timeout;
    ArmState state{};

    while (running_ && Clock::now() < deadline) {
        MoveResult motion_result = check_dart_motion(monitor_zone_index);
        if (motion_result == MoveResult::DartMoved)
            return MoveResult::DartMoved;

        if (!arm_.read_state(state)) {
            std::this_thread::sleep_for(20ms);
            continue;
        }
        if (!state.hw_ok) {
            mdlog::error("Decision", "arm hardware went offline while moving");
            return MoveResult::Failed;
        }
        if (state.reached_goal) return MoveResult::Arrived;
        std::this_thread::sleep_for(50ms);
    }

    return MoveResult::Failed;
}

DecisionModule::MoveResult DecisionModule::wait_until_near(const std::array<float, 4>& target,
                                                           float threshold,
                                                           std::chrono::milliseconds timeout,
                                                           int monitor_zone_index)
{
    using Clock = std::chrono::steady_clock;
    const auto deadline = Clock::now() + timeout;
    ArmState state{};

    while (running_ && Clock::now() < deadline) {
        MoveResult motion_result = check_dart_motion(monitor_zone_index);
        if (motion_result == MoveResult::DartMoved)
            return MoveResult::DartMoved;

        if (!arm_.read_state(state)) {
            std::this_thread::sleep_for(20ms);
            continue;
        }
        if (!state.hw_ok) {
            mdlog::error("Decision", "arm hardware went offline while moving");
            return MoveResult::Failed;
        }
        if (max_joint_delta(state.joints, target) <= threshold)
            return MoveResult::Arrived;
        std::this_thread::sleep_for(40ms);
    }

    return MoveResult::Failed;
}

DecisionModule::MoveResult DecisionModule::move_to_and_wait(const std::string& name,
                                                            float fallback_duration,
                                                            bool stop_at_goal,
                                                            int monitor_zone_index)
{
    (void)fallback_duration;

    ArmState state{};
    std::array<float, 4> target{};

    if (!arm_.get_position(name, target)) {
        mdlog::error("Decision", "unknown or unavailable position: %s", name.c_str());
        return MoveResult::Failed;
    }

    using Clock = std::chrono::steady_clock;

    const auto state_deadline = Clock::now() + 500ms;
    while (running_ && Clock::now() < state_deadline) {
        if (arm_.read_state(state))
            break;
        std::this_thread::sleep_for(10ms);
    }

    if (!state.hw_ok) {
        mdlog::error("Decision", "cannot move to %s: current arm joints unavailable",
                     name.c_str());
        return MoveResult::Failed;
    }

    const float duration = duration_for_segment(state.joints, target, config_);
    const float initial_delta = max_joint_delta(state.joints, target);
    float stream_speed = (duration > 0.05f) ? (initial_delta / duration)
                                            : config_.joint_cruise_speed_rad_s;
    if (stream_speed < 0.01f)
        stream_speed = 0.01f;

    const float arrival_threshold = stop_at_goal
        ? GOAL_REACHED_THRESH
        : config_.flythrough_joint_threshold_rad;

    if (monitor_zone_index < 0) {
        const float speedup = (config_.fast_motion_speedup > 0.1f)
            ? config_.fast_motion_speedup
            : 1.f;
        const float fast_duration = clampf(duration / speedup,
                                           0.75f,
                                           config_.max_segment_duration_s);

        mdlog::assembling("move to %s %.2fs%s",
                          name.c_str(), fast_duration, stop_at_goal ? "" : " fly-through");
        arm_.move_joints(target, fast_duration);

        if (!sleep_interruptible(std::chrono::milliseconds((1000 / ARM_LOOP_HZ) * 3)))
            return MoveResult::Failed;

        const MoveResult result = stop_at_goal
            ? wait_for_goal(seconds_to_ms(config_.move_timeout_s), -1)
            : wait_until_near(target,
                              arrival_threshold,
                              seconds_to_ms(config_.move_timeout_s),
                              -1);
        if (result != MoveResult::Arrived)
            mdlog::disturb("timeout while moving to %s", name.c_str());
        return result;
    }

    const float servo_advance = clampf(stream_speed * SERVO_LOOKAHEAD_S,
                                       MIN_SERVO_ADVANCE_RAD,
                                       MAX_SERVO_ADVANCE_RAD);

    mdlog::assembling("guarded move to %s %.2fs%s",
                      name.c_str(), duration, stop_at_goal ? "" : " fly-through");

    const auto deadline = Clock::now() + seconds_to_ms(config_.move_timeout_s);
    auto next_tick = Clock::now();

    while (running_ && Clock::now() < deadline) {
        if (!arm_.read_state(state)) {
            std::this_thread::sleep_for(5ms);
            continue;
        }

        if (!state.hw_ok) {
            mdlog::error("Decision", "arm hardware went offline while moving");
            return MoveResult::Failed;
        }

        const MoveResult motion_result = check_dart_motion(monitor_zone_index);
        if (motion_result == MoveResult::DartMoved) {
            arm_.servo_joints(state.joints);
            mdlog::disturb("dart moved; arm paused");
            return MoveResult::DartMoved;
        }

        if (max_joint_delta(state.joints, target) <= arrival_threshold) {
            if (stop_at_goal)
                arm_.servo_joints(target);
            if (stop_at_goal &&
                !sleep_interruptible(millis_to_ms(config_.waypoint_settle_ms)))
                return MoveResult::Failed;
            return MoveResult::Arrived;
        }

        const std::array<float, 4> next = step_toward(state.joints, target, servo_advance);
        arm_.servo_joints(next);

        next_tick += SERVO_MOVE_PERIOD;
        std::this_thread::sleep_until(next_tick);
    }

    mdlog::disturb("timeout while moving to %s", name.c_str());
    return MoveResult::Failed;
}

bool DecisionModule::move_to_loading_and_release()
{
    ArmState state{};
    std::array<float, 4> target{};

    if (!arm_.get_position("loading", target)) {
        mdlog::error("Decision", "unknown or unavailable position: loading");
        return false;
    }

    const bool have_state = arm_.read_state(state);
    const float duration = have_state
        ? duration_for_segment(state.joints, target, config_)
        : config_.homing_move_duration_s;

    mdlog::assembling("move to loading %.2fs", duration);
    arm_.move_joints(target, duration);

    if (!sleep_interruptible(std::chrono::milliseconds((1000 / ARM_LOOP_HZ) * 3)))
        return false;

    MoveResult result = wait_until_near(target,
                                        config_.loading_release_tolerance_rad,
                                        seconds_to_ms(config_.move_timeout_s),
                                        -1);
    if (result == MoveResult::Arrived) {
        mdlog::catch_("loading reached; opening gripper");
    } else {
        mdlog::disturb("loading not fully reached after %.1fs; opening gripper anyway",
                    config_.move_timeout_s);
    }

    arm_.release();
    return sleep_interruptible(seconds_to_ms(config_.gripper_settle_s));
}

bool DecisionModule::move_to_search_pose()
{
    if (config_.search_ready_sequence.empty()) {
        mdlog::error("Decision", "search_ready_sequence is empty");
        return false;
    }

    std::string path;
    for (const auto& pose : config_.search_ready_sequence) {
        if (!path.empty())
            path += " -> ";
        path += pose;
    }
    mdlog::searching("preparing search pose: %s", path.c_str());

    for (size_t i = 0; i < config_.search_ready_sequence.size(); ++i) {
        const bool last_step = (i + 1 == config_.search_ready_sequence.size());
        if (move_to_and_wait(config_.search_ready_sequence[i],
                             config_.homing_move_duration_s,
                             last_step) != MoveResult::Arrived)
            return false;
    }

    mdlog::searching("waiting at %s", config_.search_wait_pose.c_str());
    return true;
}

DecisionModule::MoveResult DecisionModule::check_dart_motion(int monitor_zone_index)
{
    if (!config_.monitor_dart_motion_during_pickup || monitor_zone_index < 0)
        return MoveResult::Arrived;
    if (monitor_zone_index >= static_cast<int>(config_.zones.size()))
        return MoveResult::Arrived;

    using Clock = std::chrono::steady_clock;

    Detection det{};
    if (!detection_buf_.read(det) || !det.valid) {
        pickup_outside_zone_index_ = -1;
        return MoveResult::Arrived;
    }

    const DecisionZone& expected = config_.zones[monitor_zone_index];
    if (contains_pickup_guard(expected, det, config_.pickup_motion_bbox_margin_m)) {
        pickup_outside_zone_index_ = -1;
        return MoveResult::Arrived;
    }

    const int current_zone = find_zone(det, config_.zones);
    if (current_zone == monitor_zone_index) {
        pickup_outside_zone_index_ = -1;
        return MoveResult::Arrived;
    }

    const float outside_grace_s = (expected.pickup_outside_grace_s > 0.f)
        ? expected.pickup_outside_grace_s
        : config_.pickup_motion_outside_grace_s;
    const auto now = Clock::now();
    if (pickup_outside_zone_index_ != monitor_zone_index) {
        pickup_outside_zone_index_ = monitor_zone_index;
        pickup_outside_t0_ = now;
        return MoveResult::Arrived;
    }

    if (now - pickup_outside_t0_ < seconds_to_ms(outside_grace_s))
        return MoveResult::Arrived;

    if (current_zone >= 0) {
        const DecisionZone& observed = config_.zones[current_zone];
        mdlog::disturb("dart moved during pickup: %s -> %s",
                       expected.name.c_str(), observed.name.c_str());
    } else {
        mdlog::disturb("dart moved during pickup: %s -> outside slots",
                       expected.name.c_str());
    }

    relocated_zone_index_ = current_zone;
    hold_current_joints();
    return MoveResult::DartMoved;
}

bool DecisionModule::hold_current_joints()
{
    using Clock = std::chrono::steady_clock;

    ArmState state{};
    const auto deadline = Clock::now() + 250ms;
    while (running_ && Clock::now() < deadline) {
        if (arm_.read_state(state))
            break;
        std::this_thread::sleep_for(10ms);
    }

    if (!state.hw_ok) {
        mdlog::error("Decision", "cannot hold current joints: arm state unavailable");
        return false;
    }

    arm_.servo_joints(state.joints);
    mdlog::disturb("arm paused; watching for relocation");
    sleep_interruptible(50ms);
    return true;
}

bool DecisionModule::wait_for_relocated_dart(int previous_zone_index, int& out_zone_index)
{
    using Clock = std::chrono::steady_clock;

    out_zone_index = -1;
    hold_current_joints();

    const auto window = seconds_to_ms(config_.relocation_window_s);
    const auto stable_time = seconds_to_ms(config_.relocation_stable_time_s);
    const auto outside_grace = seconds_to_ms(config_.relocation_outside_grace_s);
    const auto deadline = Clock::now() + window;

    const char* previous_name = "?";
    if (previous_zone_index >= 0 &&
        previous_zone_index < static_cast<int>(config_.zones.size())) {
        previous_name = config_.zones[previous_zone_index].name.c_str();
    }

    mdlog::disturb("relocation after %s moved: watch %.1fs, need %.1fs stable",
                   previous_name,
                   config_.relocation_window_s,
                   config_.relocation_stable_time_s);

    int candidate_zone = -1;
    auto last_sample_t = Clock::now();
    auto last_candidate_seen_t = Clock::now();
    std::chrono::milliseconds candidate_stable{0};
    Detection det{};

    while (running_ && Clock::now() < deadline) {
        const auto now = Clock::now();
        const auto sample_dt = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_sample_t);
        last_sample_t = now;

        bool have_zone_sample = false;
        int zone = -1;
        if (detection_buf_.read(det) && det.valid) {
            zone = find_zone(det, config_.zones, config_.relocation_bbox_margin_m);
            if (!config_.relocation_accept_same_slot && zone == previous_zone_index)
                zone = -1;
            have_zone_sample = (zone >= 0);
        }

        if (have_zone_sample) {
            if (zone != candidate_zone) {
                candidate_zone = zone;
                candidate_stable = std::chrono::milliseconds{0};
                last_candidate_seen_t = now;
                mdlog::disturb("relocation candidate: %s",
                               config_.zones[zone].name.c_str());
            } else {
                candidate_stable += sample_dt;
                last_candidate_seen_t = now;
                if (candidate_stable >= stable_time) {
                    out_zone_index = candidate_zone;
                    mdlog::disturb("relocation stable: %s",
                                   config_.zones[out_zone_index].name.c_str());
                    return true;
                }
            }
        } else if (candidate_zone >= 0 && now - last_candidate_seen_t > outside_grace) {
            mdlog::disturb("relocation candidate lost; resetting");
            candidate_zone = -1;
            candidate_stable = std::chrono::milliseconds{0};
        }

        std::this_thread::sleep_for(50ms);
    }

    mdlog::disturb("no stable relocation; returning to safe pose");
    return false;
}

DecisionModule::PickupResult DecisionModule::execute_pickup_trajectory(int zone_index)
{
    int current_zone_index = zone_index;

    while (running_) {
        if (current_zone_index < 0 ||
            current_zone_index >= static_cast<int>(config_.zones.size())) {
            mdlog::error("Decision", "invalid zone index: %d", current_zone_index);
            return PickupResult::Failed;
        }

        const DecisionZone& zone = config_.zones[current_zone_index];
        auto trajectory_it = config_.trajectories.find(zone.trajectory);
        if (trajectory_it == config_.trajectories.end()) {
            mdlog::error("Decision", "missing trajectory '%s' for zone '%s'",
                         zone.trajectory.c_str(), zone.name.c_str());
            return PickupResult::Failed;
        }
        const auto& steps = trajectory_it->second;

        mdlog::catch_("start %s for %s (%s mode)",
                      zone.trajectory.c_str(),
                      zone.name.c_str(),
                      config_.pickup_mode.c_str());

        bool monitor_until_prep = true;
        bool restart_with_relocated_zone = false;

        for (size_t i = 0; i < steps.size(); ++i) {
            const DecisionTrajectoryStep& step = steps[i];
            if (!running_) return PickupResult::Failed;

            if (is_loading_release_step(step)) {
                if (!move_to_loading_and_release())
                    return PickupResult::Failed;
                continue;
            }

            const bool last_step = (i + 1 == steps.size());
            const bool stop_at_goal = last_step || step.gripper != GripperAction::None;
            const int monitor_zone_index = (monitor_until_prep &&
                                            config_.monitor_dart_motion_during_pickup)
                ? current_zone_index
                : -1;

            const MoveResult move_result = move_to_and_wait(step.position,
                                                            config_.homing_move_duration_s,
                                                            stop_at_goal,
                                                            monitor_zone_index);
            if (move_result == MoveResult::DartMoved) {
                int new_zone_index = -1;
                if (wait_for_relocated_dart(current_zone_index, new_zone_index)) {
                    mdlog::disturb("relocation accepted: pass through %s, then catch %s",
                                   config_.search_wait_pose.c_str(),
                                   config_.zones[new_zone_index].name.c_str());
                    const MoveResult via_result = move_to_and_wait(config_.search_wait_pose,
                                                                    config_.homing_move_duration_s,
                                                                    false,
                                                                    -1);
                    if (via_result != MoveResult::Arrived)
                        return PickupResult::Failed;

                    current_zone_index = new_zone_index;
                    restart_with_relocated_zone = true;
                    break;
                }

                const MoveResult safe_result = move_to_and_wait(config_.search_wait_pose,
                                                                config_.homing_move_duration_s);
                return safe_result == MoveResult::Arrived
                    ? PickupResult::AbortedToSearchPose
                    : PickupResult::Failed;
            }
            if (move_result != MoveResult::Arrived)
            {
                return PickupResult::Failed;
            }

            if (monitor_until_prep && is_prep_position(step.position)) {
                monitor_until_prep = false;
                mdlog::catch_("prep reached: %s; guard off", step.position.c_str());
            }

            if (step.gripper == GripperAction::Open) {
                mdlog::catch_("open gripper at %s", step.position.c_str());
                arm_.release();
                if (!sleep_interruptible(seconds_to_ms(config_.gripper_settle_s)))
                    return PickupResult::Failed;
            } else if (step.gripper == GripperAction::Close) {
                mdlog::catch_("close gripper at %s", step.position.c_str());
                arm_.grip();
                if (!sleep_interruptible(seconds_to_ms(config_.gripper_settle_s)))
                    return PickupResult::Failed;
            }
        }

        if (restart_with_relocated_zone)
            continue;

        mdlog::catch_("pickup complete");
        return PickupResult::Complete;
    }

    return PickupResult::Failed;
}

void DecisionModule::reset_stability()
{
    stable_zone_index_ = -1;
    stable_last_seen_t_ = {};
}

bool DecisionModule::update_stability(const Detection& det,
                                      std::chrono::steady_clock::time_point now)
{
    const auto grace = seconds_to_ms(config_.search_stability_grace_s);

    if (!det.valid) {
        if (stable_zone_index_ >= 0 && now - stable_last_seen_t_ <= grace)
            return false;
        if (stable_zone_index_ >= 0)
            mdlog::disturb("lost detection; reset search stability");
        reset_stability();
        return false;
    }

    const int zone_index = find_zone(det,
                                     config_.zones,
                                     config_.search_stability_bbox_margin_m);
    if (zone_index < 0) {
        if (stable_zone_index_ >= 0 && now - stable_last_seen_t_ <= grace)
            return false;
        if (stable_zone_index_ >= 0)
            mdlog::disturb("dart left zones; reset search stability");
        reset_stability();
        return false;
    }

    const DecisionZone& zone = config_.zones[zone_index];

    if (stable_zone_index_ != zone_index) {
        stable_zone_index_ = zone_index;
        stable_t0_ = now;
        stable_last_seen_t_ = now;
        mdlog::searching("%s candidate; waiting %.1fs",
                         zone.name.c_str(), config_.stable_dart_time_s);
        return false;
    }

    stable_last_seen_t_ = now;
    if (now - stable_t0_ >= seconds_to_ms(config_.stable_dart_time_s)) {
        triggered_zone_index_ = zone_index;
        mdlog::catch_("%s stable; triggering %s",
                      zone.name.c_str(), zone.trajectory.c_str());
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
            switch (state_) {
            case State::Searching:
                mdlog::searching("active");
                break;
            case State::ExecutingTrajectory:
                mdlog::catch_("active");
                break;
            case State::Homing:
                mdlog::assembling("homing");
                break;
            case State::ResetCooldown:
                mdlog::searching("reset cooldown");
                break;
            case State::Idle:
                mdlog::system("idle");
                break;
            }
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
                mdlog::assembling("waiting for arm hardware");
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
                mdlog::error("Decision", "arm hardware unavailable; returning to idle");
                reset_stability();
                state_ = State::Idle;
                break;
            }

            if (detection_buf_.read(det) && update_stability(det, Clock::now())) {
                reset_stability();
                state_ = State::ExecutingTrajectory;
            }
            break;

        case State::ExecutingTrajectory: {
            const PickupResult pickup_result = execute_pickup_trajectory(triggered_zone_index_);
            if (pickup_result == PickupResult::Complete) {
                reset_stability();
                if (config_.reset_cooldown_s > 0.f) {
                    state_ = State::ResetCooldown;
                } else {
                    mdlog::searching("continue from current pose");
                    state_ = State::Searching;
                }
            } else if (pickup_result == PickupResult::AbortedToSearchPose) {
                reset_stability();
                state_ = State::Searching;
            } else if (running_) {
                mdlog::error("Decision", "pickup failed; returning home");
                move_to_and_wait("home", config_.homing_move_duration_s);
                state_ = State::Idle;
            }
            break;
        }

        case State::ResetCooldown:
            mdlog::searching("cooldown %.1fs before next search",
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
