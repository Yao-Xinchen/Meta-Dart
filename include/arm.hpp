#pragma once

#include "triple_buffer.hpp"
#include "types.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <map>
#include <string>
#include <thread>

// Forward-declare to avoid pulling in the full Dynamixel headers here
class DynamixelWorkbench;

// ─────────────────────────────────────────────────────────────────────────────
// ArmModule
//
// Controls the OpenManipulator-X via DynamixelWorkbench.
// Reads ArmCmd from its command buffer and publishes ArmState.
//
// Thread rate: ARM_LOOP_HZ (see config.hpp).
// ─────────────────────────────────────────────────────────────────────────────
class ArmModule {
public:
    ArmModule();
    ~ArmModule();

    // Initialise hardware and start the control thread.
    // Returns false if the arm cannot be reached.
    bool start();

    // Signal the thread to stop, send all joints to home, then join.
    void stop();

    // Command interface
    void move_joints(const std::array<float, 4>& joints, float duration = 0.f);
    bool move_to(const std::string& name, float duration = 0.f);
    void grip();
    void release();
    void idle();

    // State interface
    bool read_state(ArmState& out);

private:
    void loop();
    void execute_cmd(const ArmCmd& cmd);
    void tick_interp();
    void set_gripper(float position);
    bool goal_reached(const std::array<float, 4>& joints) const;
    bool load_positions(const char* xml_path);

    DynamixelWorkbench*     dxl_wb_    = nullptr;
    TripleBuffer<ArmCmd>    cmd_buf_;
    TripleBuffer<ArmState>  state_buf_;
    std::thread             thread_;
    std::atomic<bool>       running_{false};

    // Goal tracking
    std::array<float, 4>    goal_joints_{};
    bool                    has_goal_ = false;

    // Interpolation state
    bool                   interp_active_ = false;
    std::array<float, 4>   interp_start_  = {};
    std::array<float, 4>   interp_goal_   = {};
    std::chrono::steady_clock::time_point interp_t0_;
    float                  interp_dur_s_  = 0.f;

    // Last-read joint positions (used as interpolation start)
    std::array<float, 4>   current_joints_ = {};

    // Named positions loaded from XML
    std::map<std::string, std::array<float, 4>> positions_;
};
