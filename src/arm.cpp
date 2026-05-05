#include "arm.hpp"
#include "config.hpp"

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
    current_joints_ = {cur[0], cur[1], cur[2], cur[3]};

    // Read gripper raw encoder unit directly so we can hold it without
    // converting through radians (avoids any Drive Mode sign ambiguity).
    int32_t cur_grip_unit = 0;
    dxl_wb_->itemRead(DXL_ID_GRIPPER, "Present_Position", &cur_grip_unit, &log);

    // Step 3: Extended Position Mode allows multi-turn range (joints travel
    //         beyond ±π). Restore current position before torque-on so the
    //         arm doesn't snap to the default Goal Position (0).
    for (int i = 0; i < 4; ++i) {
        dxl_wb_->setExtendedPositionControlMode(JOINT_IDS[i], &log);
        dxl_wb_->goalPosition(JOINT_IDS[i], cur[i], &log);
        dxl_wb_->torqueOn(JOINT_IDS[i], &log);
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

        // ── Send interpolated setpoint ────────────────────────────────────
        tick_interp();

        // ── Read present positions and publish state ───────────────────────
        ArmState state{};
        state.hw_ok = (dxl_wb_ != nullptr);

        for (int i = 0; i < 4; ++i) {
            float rad = 0.f;
            if (dxl_wb_) dxl_wb_->getRadian(JOINT_IDS[i], &rad);
            state.joints[i] = rad;
        }
        current_joints_ = state.joints;
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
        interp_active_ = false;
        has_goal_      = false;
        break;

    case ArmCmd::Type::MoveJoint:
        interp_start_  = current_joints_;
        interp_goal_   = cmd.joints;
        interp_t0_     = std::chrono::steady_clock::now();
        interp_dur_s_  = (cmd.duration > 0.f) ? cmd.duration : 2.0f;
        interp_active_ = true;
        goal_joints_   = cmd.joints;
        has_goal_      = true;
        break;

    case ArmCmd::Type::Grip:
        set_gripper(GRIPPER_CLOSED);
        interp_active_ = false;
        has_goal_      = false;
        break;

    case ArmCmd::Type::Release:
        set_gripper(GRIPPER_OPEN);
        interp_active_ = false;
        has_goal_      = false;
        break;
    }
}

void ArmModule::tick_interp() {
    if (!interp_active_ || !dxl_wb_) return;
    using Clock = std::chrono::steady_clock;
    float elapsed = std::chrono::duration<float>(Clock::now() - interp_t0_).count();
    float t = (interp_dur_s_ > 0.f) ? (elapsed / interp_dur_s_) : 1.f;
    if (t >= 1.f) {
        t = 1.f;
        interp_active_ = false;
    }
    const char* log = nullptr;
    for (int i = 0; i < 4; ++i) {
        float pos = interp_start_[i] + t * (interp_goal_[i] - interp_start_[i]);
        dxl_wb_->goalPosition(JOINT_IDS[i], pos, &log);
    }
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
