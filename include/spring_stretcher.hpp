#pragma once

#include "triple_buffer.hpp"
#include "types.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>

struct can_frame;

// -----------------------------------------------------------------------------
// SpringStretcherModule
//
// Drives two DJI M3508 motors over SocketCAN to stretch the launcher springs.
// This module only handles the motor sequence:
//   Stretch springs -> retract motors to home.
// The mechanical trigger/release is intentionally left for a separate module.
// -----------------------------------------------------------------------------
class SpringStretcherModule {
public:
    SpringStretcherModule();
    ~SpringStretcherModule();

    bool start();
    void stop();

    void stretch();
    void retract();
    void idle();

    bool read_state(SpringStretcherState& out);

private:
    struct Motor {
        int id = 1;
        float direction = 1.f;

        float zero = 0.f;
        float raw_position = 0.f;
        float position = 0.f;
        float velocity = 0.f;
        float current = 0.f;

        float target = 0.f;
        float command_current = 0.f;
        float pos_error = 0.f;
        float vel_error = 0.f;
        float pos_i = 0.f;
        float vel_i = 0.f;

        bool zeroed = false;
        bool feedback_ok = false;
        bool raw_position_valid = false;
        bool calibration_failed = false;

        std::chrono::steady_clock::time_point calibration_started_at{};
        std::chrono::steady_clock::time_point last_moving_at{};
    };

    void loop();
    void execute_cmd(const SpringStretcherCmd& cmd);
    bool open_can();
    void close_can();
    bool read_feedback();
    void process_feedback(const can_frame& frame);
    void send_current_commands();
    void update_control();
    void update_calibration();
    void publish_state();
    bool calibrated() const;
    bool goal_reached() const;

    static float wrap_delta(float angle);
    static float clamp(float value, float limit);

    TripleBuffer<SpringStretcherCmd>   cmd_buf_;
    TripleBuffer<SpringStretcherState> state_buf_;
    std::thread                thread_;
    std::atomic<bool>          running_{false};

    std::array<Motor, 2> motors_;
    mutable std::mutex   motor_mtx_;

    int  can_socket_ = -1;
    bool hw_ok_ = false;
    bool calibration_active_ = false;
    bool has_goal_ = false;
    bool retract_after_stretch_ = false;
    std::chrono::steady_clock::time_point stretch_reached_at_{};
};
