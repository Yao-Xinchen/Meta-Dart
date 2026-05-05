#include "arm.hpp"
#include "config.hpp"

#include <DynamixelWorkbench.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
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

static float clamp01(float t)
{
    if (t < 0.f) return 0.f;
    if (t > 1.f) return 1.f;
    return t;
}

// Quintic S-curve: zero velocity and zero acceleration at both ends.
// This makes each joint ease in and ease out instead of changing speed abruptly.
static float smoothstep_quintic(float t)
{
    t = clamp01(t);
    return t * t * t * (t * (t * 6.f - 15.f) + 10.f);
}

static void write_joint_softness(DynamixelWorkbench* wb, uint8_t id)
{
    const char* log = nullptr;

    if (!wb->itemWrite(id, "Profile_Acceleration", JOINT_PROFILE_ACCELERATION, &log))
        std::fprintf(stderr, "[Arm] warn: failed to set Profile_Acceleration for ID %u: %s\n",
                     id, log ? log : "?");
    if (!wb->itemWrite(id, "Profile_Velocity", JOINT_PROFILE_VELOCITY, &log))
        std::fprintf(stderr, "[Arm] warn: failed to set Profile_Velocity for ID %u: %s\n",
                     id, log ? log : "?");

    if (!wb->itemWrite(id, "Position_P_Gain", JOINT_POSITION_P_GAIN, &log))
        std::fprintf(stderr, "[Arm] warn: failed to set Position_P_Gain for ID %u: %s\n",
                     id, log ? log : "?");
    if (!wb->itemWrite(id, "Position_I_Gain", JOINT_POSITION_I_GAIN, &log))
        std::fprintf(stderr, "[Arm] warn: failed to set Position_I_Gain for ID %u: %s\n",
                     id, log ? log : "?");
    if (!wb->itemWrite(id, "Position_D_Gain", JOINT_POSITION_D_GAIN, &log))
        std::fprintf(stderr, "[Arm] warn: failed to set Position_D_Gain for ID %u: %s\n",
                     id, log ? log : "?");
}

static constexpr std::chrono::microseconds LOOP_PERIOD{1'000'000 / ARM_LOOP_HZ};
static constexpr std::chrono::milliseconds GOAL_WRITE_PERIOD{20};

ArmModule::ArmModule()  = default;
ArmModule::~ArmModule() { stop(); }

// ─────────────────────────────────────────────────────────────────────────────
// Named positions
// ─────────────────────────────────────────────────────────────────────────────
bool ArmModule::load_positions(const char* xml_path)
{
    std::ifstream f(xml_path);
    if (!f) {
        std::fprintf(stderr, "[Arm] cannot open positions file: %s\n", xml_path);
        return false;
    }
    auto get_attr = [](const std::string& line, const std::string& attr) -> std::string {
        auto pos = line.find(attr + "=\"");
        if (pos == std::string::npos) return {};
        pos += attr.size() + 2;
        return line.substr(pos, line.find('"', pos) - pos);
    };
    std::string line;
    while (std::getline(f, line)) {
        if (line.find("<position") == std::string::npos) continue;
        std::string name = get_attr(line, "name");
        if (name.empty()) continue;
        std::array<float, 4> j{};
        j[0] = std::stof(get_attr(line, "j1"));
        j[1] = std::stof(get_attr(line, "j2"));
        j[2] = std::stof(get_attr(line, "j3"));
        j[3] = std::stof(get_attr(line, "j4"));
        positions_[name] = j;
    }
    std::printf("[Arm] loaded %zu position(s) from %s\n", positions_.size(), xml_path);
    return true;
}

bool ArmModule::move_to(const std::string& name, float duration)
{
    auto it = positions_.find(name);
    if (it == positions_.end()) {
        std::fprintf(stderr, "[Arm] unknown position: '%s'\n", name.c_str());
        return false;
    }
    move_joints(it->second, duration);
    return true;
}

bool ArmModule::get_position(const std::string& name, std::array<float, 4>& out) const
{
    auto it = positions_.find(name);
    if (it == positions_.end())
        return false;
    out = it->second;
    return true;
}

void ArmModule::move_joints(const std::array<float, 4>& joints, float duration)
{
    ArmCmd cmd;
    cmd.type     = ArmCmd::Type::MoveJoint;
    cmd.joints   = joints;
    cmd.duration = duration;
    cmd_buf_.write(cmd);
}

void ArmModule::servo_joints(const std::array<float, 4>& joints)
{
    ArmCmd cmd;
    cmd.type   = ArmCmd::Type::ServoJoint;
    cmd.joints = joints;
    cmd_buf_.write(cmd);
}

void ArmModule::grip()
{
    ArmCmd cmd;
    cmd.type = ArmCmd::Type::Grip;
    cmd_buf_.write(cmd);
}

void ArmModule::release()
{
    ArmCmd cmd;
    cmd.type = ArmCmd::Type::Release;
    cmd_buf_.write(cmd);
}

void ArmModule::idle()
{
    ArmCmd cmd;
    cmd.type = ArmCmd::Type::Idle;
    cmd_buf_.write(cmd);
}

bool ArmModule::read_state(ArmState& out)
{
    return state_buf_.read(out);
}

// ─────────────────────────────────────────────────────────────────────────────
bool ArmModule::start() {
    load_positions(POSITIONS_XML_PATH);

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
    bool all_motors_ok = true;
    for (uint8_t id : ALL_IDS) {
        if (!dxl_wb_->ping(id, &model, &log)) {
            std::fprintf(stderr, "[Arm] ping failed for ID %u: %s\n", id, log ? log : "?");
            all_motors_ok = false;
        }
    }

    if (!all_motors_ok) {
        std::fprintf(stderr, "[Arm] one or more motors reported errors; refusing to enable torque\n");
        delete dxl_wb_;
        dxl_wb_ = nullptr;

        ArmState bad{};
        bad.hw_ok = false;
        state_buf_.write(bad);
        return false;
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
        write_joint_softness(dxl_wb_, JOINT_IDS[i]);
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
        last_goal_sent_valid_ = false;
        last_goal_write_time_ = interp_t0_;
        break;

    case ArmCmd::Type::ServoJoint: {
        interp_active_ = false;
        goal_joints_   = cmd.joints;
        has_goal_      = true;

        if (!dxl_wb_)
            break;

        const char* log = nullptr;
        for (int i = 0; i < 4; ++i) {
            dxl_wb_->goalPosition(JOINT_IDS[i], cmd.joints[i], &log);
            last_goal_sent_[i] = cmd.joints[i];
        }
        last_goal_sent_valid_ = true;
        last_goal_write_time_ = std::chrono::steady_clock::now();
        break;
    }

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
    const bool final_setpoint = (t >= 1.f);
    float s = smoothstep_quintic(t);
    const auto now = Clock::now();
    const bool write_period_elapsed =
        !last_goal_sent_valid_ || (now - last_goal_write_time_) >= GOAL_WRITE_PERIOD;
    const char* log = nullptr;
    bool wrote_any = false;
    for (int i = 0; i < 4; ++i) {
        float pos = interp_start_[i] + s * (interp_goal_[i] - interp_start_[i]);
        bool should_write = final_setpoint || !last_goal_sent_valid_ ||
                            (write_period_elapsed &&
                             std::abs(pos - last_goal_sent_[i]) >= JOINT_GOAL_WRITE_DEADBAND);
        if (should_write) {
            dxl_wb_->goalPosition(JOINT_IDS[i], pos, &log);
            last_goal_sent_[i] = pos;
            wrote_any = true;
        }
    }
    if (wrote_any)
        last_goal_write_time_ = now;
    last_goal_sent_valid_ = true;
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
