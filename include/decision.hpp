#pragma once

#include "triple_buffer.hpp"
#include "types.hpp"

#include <atomic>
#include <chrono>
#include <thread>

// ─────────────────────────────────────────────────────────────────────────────
// DecisionModule — pick-and-place state machine
//
//  MOVING_HOME ──► SEARCHING ──► MOVING_TO_PICK_HOVER ──► MOVING_TO_PICK
//       ▲                                                        │
//       └── RELEASING ◄── MOVING_TO_PLACE ◄── LIFTING ◄── GRIPPING
//
// The module waits until VisionModule reports a valid Detection inside a known
// pick region. Region 1 maps to calibrated fixed joint angles. ArmModule still
// publishes FK in ArmState so these fixed angles can be verified on hardware.
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
        MovingHome,
        Searching,
        MovingToPickHover,
        MovingToPick,
        Gripping,
        Lifting,
        MovingToPlace,
        Releasing,
    };

    enum class PickSlot {
        None,
        Position1,
        Position2,
        Position3,
        Position4,
    };

    void loop();
    PickSlot classify_detection(const Detection& det) const;
    bool slot_is_calibrated(PickSlot slot) const;
    const std::array<float, 4>& slot_hover_joints(PickSlot slot) const;
    const std::array<float, 4>& slot_pick_joints(PickSlot slot) const;
    void report_detection_region(const Detection& det, PickSlot slot);
    void start_pick(PickSlot slot);
    void transition_to(State next);
    void send_cmd(const ArmCmd& cmd);
    const char* pick_slot_name(PickSlot slot) const;
    const char* state_name(State s) const;

    TripleBuffer<Detection>& detection_buf_;
    TripleBuffer<ArmState>&  arm_state_buf_;
    TripleBuffer<ArmCmd>&    arm_cmd_buf_;

    std::thread      thread_;
    std::atomic<bool> running_{false};
    State             state_ = State::Idle;
    PickSlot          active_slot_ = PickSlot::None;
    PickSlot          last_reported_slot_ = PickSlot::None;
    PickSlot          last_warned_uncalibrated_slot_ = PickSlot::None;
    std::chrono::steady_clock::time_point state_entered_{};
    bool              last_reported_detection_valid_ = false;
};
