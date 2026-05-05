#pragma once

#include "triple_buffer.hpp"
#include "types.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>

class ArmModule;

// ─────────────────────────────────────────────────────────────────────────────
// DecisionModule — stable-detection-triggered pickup state machine
//
// The module moves the arm to the search wait pose, then watches the latest
// Detection from VisionModule.  If the dart remains inside one configured XY
// bounding box for long enough, it runs that zone's pickup trajectory.  During
// arm motion, camera updates are ignored until reset finishes and the arm has
// returned to the search wait pose.
// ─────────────────────────────────────────────────────────────────────────────
class DecisionModule {
public:
    DecisionModule(TripleBuffer<Detection>& detection_buf, ArmModule& arm);
    ~DecisionModule();

    bool start();
    void stop();

private:
    enum class State {
        Idle,
        Homing,
        Searching,
        ExecutingTrajectory,
        ResetCooldown,
    };

    void loop();
    const char* state_name(State s) const;
    bool sleep_interruptible(std::chrono::milliseconds duration) const;
    bool wait_for_goal(std::chrono::seconds timeout) const;
    bool wait_until_near(const std::array<float, 4>& target,
                         float threshold,
                         std::chrono::seconds timeout) const;
    bool move_to_and_wait(const std::string& name, float duration, bool stop_at_goal = true);
    bool move_to_search_pose();
    bool execute_pickup_trajectory(int zone_index);
    void reset_stability();
    bool update_stability(const Detection& det,
                          std::chrono::steady_clock::time_point now);

    TripleBuffer<Detection>& detection_buf_;
    ArmModule&               arm_;

    std::thread       thread_;
    std::atomic<bool> running_{false};
    State             state_ = State::Idle;

    std::chrono::steady_clock::time_point stable_t0_{};
    int stable_zone_index_ = -1;
    int triggered_zone_index_ = -1;
};
