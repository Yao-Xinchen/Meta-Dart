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
static constexpr uint8_t ALL_IDS[5]    = {DXL_ID_J1, DXL_ID_J2, DXL_ID_J3, DXL_ID_J4, DXL_ID_GRIPPER};

// XM430 LED: address 65, range 0-1, physical LED colour is red.
static constexpr uint8_t LED_OFF = 0;
static constexpr uint8_t LED_ON  = 1;

// Convert gripper position in metres to raw encoder units,
// using the same scale/offset as the OpenManipulator ros2_control plugin.
static int32_t gripper_m_to_unit(float meters)
{
    return static_cast<int32_t>((meters - GRIPPER_OFFSET) / GRIPPER_SCALE);
}

static void set_leds(DynamixelWorkbench* wb, uint8_t color)
{
    const char* log = nullptr;
    for (uint8_t id : ALL_IDS)
        wb->itemWrite(id, "LED", color, &log);
}

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

    // Step 1: Ping all motors so DynamixelWorkbench discovers their models.
    //         getRadian() needs the model to know which register to read.
    uint16_t model = 0;
    for (uint8_t id : ALL_IDS) {
        if (!dxl_wb_->ping(id, &model, &log))
            std::fprintf(stderr, "[Arm] ping failed for ID %u: %s\n", id, log ? log : "?");
    }

    // Step 2: Read current positions BEFORE enabling torque so the arm
    //         doesn't snap to the default Goal Position (0) on init.
    float cur[4] = {};
    for (int i = 0; i < 4; ++i)
        dxl_wb_->getRadian(JOINT_IDS[i], &cur[i]);

    // Read gripper raw encoder unit directly so we can hold it without
    // converting through radians (avoids any Drive Mode sign ambiguity).
    int32_t cur_grip_unit = 0;
    dxl_wb_->itemRead(DXL_ID_GRIPPER, "Present_Position", &cur_grip_unit, &log);

    // Step 3: Enable joint mode and immediately restore the goal position.
    for (int i = 0; i < 4; ++i) {
        dxl_wb_->jointMode(JOINT_IDS[i], 0, 0, &log);
        dxl_wb_->goalPosition(JOINT_IDS[i], cur[i], &log);
    }
    // Gripper: current-based position control (mode 5), Drive Mode 5
    // (reverse direction), Goal Current 200 — matches OpenManipulator reference.
    dxl_wb_->itemWrite(DXL_ID_GRIPPER, "Drive_Mode", 5, &log);
    dxl_wb_->currentBasedPositionMode(DXL_ID_GRIPPER, GRIPPER_CURRENT, &log);
    dxl_wb_->itemWrite(DXL_ID_GRIPPER, "Goal_Position", cur_grip_unit, &log);

    set_leds(dxl_wb_, LED_ON);

    running_ = true;
    thread_  = std::thread(&ArmModule::loop, this);
    return true;
}

void ArmModule::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();

    if (dxl_wb_) {
        const char* log = nullptr;
        for (uint8_t id : ALL_IDS)
            dxl_wb_->torqueOff(id, &log);
        set_leds(dxl_wb_, LED_OFF);
        delete dxl_wb_;
        dxl_wb_ = nullptr;
    }
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

void ArmModule::set_gripper(float position_m) {
    if (!dxl_wb_) return;
    const char* log = nullptr;
    dxl_wb_->itemWrite(DXL_ID_GRIPPER, "Goal_Position",
                       gripper_m_to_unit(position_m), &log);
}

bool ArmModule::goal_reached(const std::array<float, 4>& joints) const {
    for (int i = 0; i < 4; ++i) {
        if (std::abs(joints[i] - goal_joints_[i]) > GOAL_REACHED_THRESH)
            return false;
    }
    return true;
}
