#pragma once

#include "triple_buffer.hpp"
#include "types.hpp"

#include <atomic>
#include <thread>

// ─────────────────────────────────────────────────────────────────────────────
// DecisionModule — pick-and-place state machine
//
//  IDLE ──► SEARCHING ──► MOVING_TO_PICK ──► PICKING
//                                                │
//  IDLE ◄── PLACING ◄── MOVING_TO_PLACE ◄────────┘
//
// The module waits in SEARCHING until VisionModule provides a valid Detection.
// It then uses IK to compute the pick joint angles, commands the arm, waits
// for goal_reached, closes the gripper, moves to the PLACE pose, and releases.
//
// Named poses (HOME, PLACE) are left as zero-initialised stubs to be filled
// in once the physical setup is calibrated.
// ─────────────────────────────────────────────────────────────────────────────
class DecisionModule {
public:
    DecisionModule(TripleBuffer<Detection>& detection_buf,
                   TripleBuffer<ArmState>&  arm_state_buf,
                   TripleBuffer<ArmCmd>&    arm_cmd_buf);
    ~DecisionModule();

    bool start();
    void stop();

private:
    enum class State {
        Idle,
        Searching,
        MovingToPick,
        Picking,
        MovingToPlace,
        Placing,
    };

    void loop();
    void send_cmd(const ArmCmd& cmd);
    const char* state_name(State s) const;

    TripleBuffer<Detection>& detection_buf_;
    TripleBuffer<ArmState>&  arm_state_buf_;
    TripleBuffer<ArmCmd>&    arm_cmd_buf_;

    std::thread      thread_;
    std::atomic<bool> running_{false};
    State             state_ = State::Idle;
};
