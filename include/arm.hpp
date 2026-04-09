#pragma once

#include "triple_buffer.hpp"
#include "types.hpp"

#include <atomic>
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

    TripleBuffer<ArmCmd>&   cmd_buf()   { return cmd_buf_; }
    TripleBuffer<ArmState>& state_buf() { return state_buf_; }

private:
    void loop();
    void execute_cmd(const ArmCmd& cmd);
    void move_joints(const std::array<float, 4>& joints);
    void set_gripper(float position);
    bool goal_reached(const std::array<float, 4>& joints) const;

    DynamixelWorkbench*     dxl_wb_    = nullptr;
    TripleBuffer<ArmCmd>    cmd_buf_;
    TripleBuffer<ArmState>  state_buf_;
    std::thread             thread_;
    std::atomic<bool>       running_{false};

    // Last commanded joint angles for goal-reached check
    std::array<float, 4>   goal_joints_{};
    bool                    has_goal_ = false;
};
