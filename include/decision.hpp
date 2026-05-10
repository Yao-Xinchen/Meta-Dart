#pragma once

#include "triple_buffer.hpp"
#include "types.hpp"

#include <atomic>
#include <thread>

class ArmModule;
class SpringStretcherModule;

// ─────────────────────────────────────────────────────────────────────────────
// DecisionModule — pick-and-place state machine
//
//  IDLE -> SEARCHING -> MOVING_TO_PICK -> PICKING
//                                           |
//  READY_TO_SHOOT <- STRETCHING_SPRINGS <- PLACING <- MOVING_TO_PLACE
//
// The module waits in SEARCHING until VisionModule provides a valid Detection.
// It then commands the arm by name, waits for goal_reached, closes the gripper,
// moves to the PLACE pose, and releases.
// ─────────────────────────────────────────────────────────────────────────────
class DecisionModule {
public:
    DecisionModule(TripleBuffer<Detection>& detection_buf,
                   ArmModule& arm,
                   SpringStretcherModule& spring_stretcher);
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
        StretchingSprings,
        ReadyToShoot,
    };

    void loop();
    const char* state_name(State s) const;

    TripleBuffer<Detection>& detection_buf_;
    ArmModule&               arm_;
    SpringStretcherModule&   spring_stretcher_;

    std::thread       thread_;
    std::atomic<bool> running_{false};
    State             state_ = State::Idle;
};
