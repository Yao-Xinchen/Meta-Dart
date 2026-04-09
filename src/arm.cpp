#include "arm.hpp"
#include "config.hpp"
#include "kinematics.hpp"

#include <DynamixelWorkbench.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────
static constexpr uint8_t JOINT_IDS[4]  = {DXL_ID_J1, DXL_ID_J2, DXL_ID_J3, DXL_ID_J4};
static constexpr std::chrono::microseconds LOOP_PERIOD{1'000'000 / ARM_LOOP_HZ};

ArmModule::ArmModule()  = default;
ArmModule::~ArmModule() { stop(); }

// ─────────────────────────────────────────────────────────────────────────────
bool ArmModule::start() {
    const char* log = nullptr;
    dxl_wb_ = new DynamixelWorkbench();

    if (!dxl_wb_->init(DXL_PORT, DXL_BAUD, &log)) {
        std::fprintf(stderr, "[Arm] init failed: %s\n", log ? log : "?");
        delete dxl_wb_;
        dxl_wb_ = nullptr;

        // Publish a hw_ok=false state so the decision module knows
        ArmState bad{};
        bad.hw_ok = false;
        state_buf_.write(bad);
        return false;
    }

    // Ping and configure each joint
    uint16_t model = 0;
    for (uint8_t id : JOINT_IDS) {
        if (!dxl_wb_->ping(id, &model, &log)) {
            std::fprintf(stderr, "[Arm] ping failed for ID %u: %s\n", id, log ? log : "?");
        }
        dxl_wb_->jointMode(id, 0, 0, &log);  // position control, no vel/acc profile
    }

    // Gripper
    if (dxl_wb_->ping(DXL_ID_GRIPPER, &model, &log)) {
        dxl_wb_->jointMode(DXL_ID_GRIPPER, 0, 0, &log);
        set_gripper(GRIPPER_OPEN);
    }

    running_ = true;
    thread_  = std::thread(&ArmModule::loop, this);
    return true;
}

void ArmModule::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    delete dxl_wb_;
    dxl_wb_ = nullptr;
}

// ─────────────────────────────────────────────────────────────────────────────
// Control loop
// ─────────────────────────────────────────────────────────────────────────────
void ArmModule::loop() {
    using Clock = std::chrono::steady_clock;
    ArmCmd cmd;

    while (running_) {
        auto t0 = Clock::now();

        // ── Consume latest command ────────────────────────────────────────
        if (cmd_buf_.read(cmd)) {
            execute_cmd(cmd);
        }

        // ── Read present positions and publish state ───────────────────────
        ArmState state{};
        state.hw_ok = (dxl_wb_ != nullptr);

        for (int i = 0; i < 4; ++i) {
            float rad = 0.f;
            if (dxl_wb_) dxl_wb_->getRadian(JOINT_IDS[i], &rad);
            state.joints[i] = rad;
        }

        Pose ee = fk(state.joints);
        state.px = ee.x;
        state.py = ee.y;
        state.pz = ee.z;
        state.reached_goal = has_goal_ && goal_reached(state.joints);

        state_buf_.write(state);

        // ── Sleep for the remainder of the loop period ────────────────────
        std::this_thread::sleep_until(t0 + LOOP_PERIOD);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void ArmModule::execute_cmd(const ArmCmd& cmd) {
    switch (cmd.type) {
    case ArmCmd::Type::Idle:
        has_goal_ = false;
        break;

    case ArmCmd::Type::MoveJoint:
        move_joints(cmd.joints);
        break;

    case ArmCmd::Type::MovePose: {
        auto result = ik(Pose{cmd.px, cmd.py, cmd.pz, cmd.pitch});
        if (result) {
            move_joints(*result);
        } else {
            std::fprintf(stderr, "[Arm] IK failed for (%.3f, %.3f, %.3f)\n",
                         cmd.px, cmd.py, cmd.pz);
        }
        break;
    }

    case ArmCmd::Type::Grip:
        set_gripper(GRIPPER_CLOSED);
        has_goal_ = false;
        break;

    case ArmCmd::Type::Release:
        set_gripper(GRIPPER_OPEN);
        has_goal_ = false;
        break;
    }
}

void ArmModule::move_joints(const std::array<float, 4>& joints) {
    if (!dxl_wb_) return;
    const char* log = nullptr;
    for (int i = 0; i < 4; ++i) {
        dxl_wb_->goalPosition(JOINT_IDS[i], joints[i], &log);
    }
    goal_joints_ = joints;
    has_goal_    = true;
}

void ArmModule::set_gripper(float position) {
    if (!dxl_wb_) return;
    const char* log = nullptr;
    dxl_wb_->goalPosition(DXL_ID_GRIPPER, position, &log);
}

bool ArmModule::goal_reached(const std::array<float, 4>& joints) const {
    for (int i = 0; i < 4; ++i) {
        if (std::abs(joints[i] - goal_joints_[i]) > GOAL_REACHED_THRESH)
            return false;
    }
    return true;
}
