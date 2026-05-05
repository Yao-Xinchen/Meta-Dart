#include "decision.hpp"
#include "arm.hpp"
#include "config.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

using namespace std::chrono_literals;

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
// State machine
// ─────────────────────────────────────────────────────────────────────────────
void DecisionModule::loop() {
    constexpr auto period = std::chrono::microseconds(1'000'000 / DECISION_LOOP_HZ);
    using Clock = std::chrono::steady_clock;

    Detection det;
    ArmState  arm_state{};
    State     prev_state = State::Idle;

    arm_.move_to("home");
    state_ = State::Searching;

    while (running_) {
        auto t0 = Clock::now();

        detection_buf_.read(det);
        arm_.read_state(arm_state);

        if (state_ != prev_state) {
            std::printf("[Decision] → %s\n", state_name(state_));
            prev_state = state_;
        }

        switch (state_) {

        case State::Idle:
            break;

        case State::Searching:
            if (!arm_state.hw_ok) break;
            if (det.valid) {
                arm_.move_to("pick");
                state_ = State::MovingToPick;
            }
            break;

        case State::MovingToPick:
            if (arm_state.reached_goal) {
                arm_.grip();
                state_ = State::Picking;
            }
            break;

        case State::Picking:
            if (arm_state.reached_goal) {
                arm_.move_to("place");
                state_ = State::MovingToPlace;
            }
            break;

        case State::MovingToPlace:
            if (arm_state.reached_goal) {
                arm_.release();
                state_ = State::Placing;
            }
            break;

        case State::Placing:
            arm_.move_to("home");
            state_ = State::Searching;
            break;
        }

        std::this_thread::sleep_until(t0 + period);
    }
}

const char* DecisionModule::state_name(State s) const {
    switch (s) {
    case State::Idle:           return "IDLE";
    case State::Searching:      return "SEARCHING";
    case State::MovingToPick:   return "MOVING_TO_PICK";
    case State::Picking:        return "PICKING";
    case State::MovingToPlace:  return "MOVING_TO_PLACE";
    case State::Placing:        return "PLACING";
    default:                    return "?";
    }
}
