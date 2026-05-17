#pragma once

#include "triple_buffer.hpp"
#include "types.hpp"

#include <atomic>
#include <chrono>
#include <thread>

// -----------------------------------------------------------------------------
// TriggerModule
//
// Command-only v1 interface for the launch trigger motor. The motor is assumed to
// have two commanded positions:
//   high/hold   - bar stays high and holds the spring
//   low/release - wire pulls the bar down and releases the spring
//
// TODO: replace the stubbed write_trigger_position() implementation with the
// actual trigger motor driver once the hardware interface is selected.
// -----------------------------------------------------------------------------
class TriggerModule {
public:
    TriggerModule();
    ~TriggerModule();

    bool start();
    void stop();

    void hold();
    void release();

    bool read_state(TriggerState& out);

private:
    void loop();
    void execute_cmd(const TriggerCmd& cmd);
    void write_trigger_position(float position_rad);
    void publish_state();

    TripleBuffer<TriggerCmd>   cmd_buf_;
    TripleBuffer<TriggerState> state_buf_;
    std::thread                thread_;
    std::atomic<bool>          running_{false};

    TriggerState::Position commanded_position_ = TriggerState::Position::Unknown;
    std::chrono::steady_clock::time_point command_time_{};
    bool hw_ok_ = true;
};
