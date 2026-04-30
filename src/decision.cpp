#include "decision.hpp"
#include "config.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────────────────────
// Named poses — read qpos from hardware and fill these in before deploying.
//
// HOME:  safe resting position with the arm upright.
// PICK:  position above the dart pick-up point.
// PLACE: position above the track loading point where the dart is released.
// ─────────────────────────────────────────────────────────────────────────────
static const ArmCmd HOME_CMD = {
    .type     = ArmCmd::Type::MoveJoint,
    .joints   = {0.f, -1.05f, 0.35f, 0.70f},  // TODO: tune on hardware
    .duration = 2.0f,
};

static const ArmCmd PICK_CMD = {
    .type     = ArmCmd::Type::MoveJoint,
    .joints   = {0.f, 0.f, 0.f, 0.f},  // TODO: read from hardware at pick pose
    .duration = 2.0f,
};

static const ArmCmd PLACE_CMD = {
    .type     = ArmCmd::Type::MoveJoint,
    .joints   = {0.f, 0.f, 0.f, 0.f},  // TODO: read from hardware at place pose
    .duration = 2.0f,
};

// ─────────────────────────────────────────────────────────────────────────────

DecisionModule::DecisionModule(TripleBuffer<Detection>& detection_buf,
                               TripleBuffer<ArmState>&  arm_state_buf,
                               TripleBuffer<ArmCmd>&    arm_cmd_buf)
    : detection_buf_(detection_buf)
    , arm_state_buf_(arm_state_buf)
    , arm_cmd_buf_(arm_cmd_buf)
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

void DecisionModule::send_cmd(const ArmCmd& cmd) {
    arm_cmd_buf_.write(cmd);
}

// ─────────────────────────────────────────────────────────────────────────────
// State machine
// ─────────────────────────────────────────────────────────────────────────────
void DecisionModule::loop() {
    constexpr auto period = std::chrono::microseconds(1'000'000 / DECISION_LOOP_HZ);
    using Clock = std::chrono::steady_clock;

    Detection  det;
    ArmState   arm;
    State      prev_state = State::Idle;

    // Start by going home
    send_cmd(HOME_CMD);
    state_ = State::Searching;

    while (running_) {
        auto t0 = Clock::now();

        // Read latest data (non-blocking)
        detection_buf_.read(det);
        arm_state_buf_.read(arm);

        if (state_ != prev_state) {
            std::printf("[Decision] → %s\n", state_name(state_));
            prev_state = state_;
        }

        switch (state_) {

        case State::Idle:
            // Nothing to do — wait for external trigger or restart
            break;

        case State::Searching:
            if (!arm.hw_ok) break;  // arm not ready
            if (det.valid) {
                send_cmd(PICK_CMD);
                state_ = State::MovingToPick;
            }
            break;

        case State::MovingToPick:
            if (arm.reached_goal) {
                ArmCmd grip{.type = ArmCmd::Type::Grip};
                send_cmd(grip);
                state_ = State::Picking;
            }
            break;

        case State::Picking:
            // Give the gripper time to close, then lift slightly and go to place
            // A more robust approach: check gripper current/load
            if (arm.reached_goal) {
                send_cmd(PLACE_CMD);
                state_ = State::MovingToPlace;
            }
            break;

        case State::MovingToPlace:
            if (arm.reached_goal) {
                ArmCmd release{.type = ArmCmd::Type::Release};
                send_cmd(release);
                state_ = State::Placing;
            }
            break;

        case State::Placing:
            // After releasing, return home and search for the next dart
            send_cmd(HOME_CMD);
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
