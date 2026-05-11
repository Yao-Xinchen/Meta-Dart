#include "spring_stretcher.hpp"
#include "config.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

using namespace std::chrono_literals;

static constexpr float PI = 3.14159265358979323846f;
static constexpr float M3508_ENCODER_TO_RAD = 2.0f * PI / 8192.0f;
static constexpr float M3508_RPM_TO_RAD_S = PI / 30.0f;
static constexpr float M3508_CURRENT_SCALE_A = 20.0f / 16384.0f;
static constexpr float M3508_CMD_SCALE = 16384.0f / 20.0f;
static constexpr auto LOOP_PERIOD =
    std::chrono::microseconds(1'000'000 / SPRING_STRETCHER_LOOP_HZ);

SpringStretcherModule::SpringStretcherModule()
{
    motors_[0].id = SPRING_STRETCHER_LEFT_ID;
    motors_[0].direction = SPRING_STRETCHER_LEFT_DIR;
    motors_[1].id = SPRING_STRETCHER_RIGHT_ID;
    motors_[1].direction = SPRING_STRETCHER_RIGHT_DIR;
}

SpringStretcherModule::~SpringStretcherModule()
{
    stop();
}

bool SpringStretcherModule::start()
{
    hw_ok_ = open_can();
    if (!hw_ok_) {
        SpringStretcherState bad{};
        bad.hw_ok = false;
        state_buf_.write(bad);
        return false;
    }

    calibration_active_ = true;
    std::printf("[SpringStretcher] calibrating hooks toward top stops\n");

    running_ = true;
    thread_ = std::thread(&SpringStretcherModule::loop, this);
    return true;
}

void SpringStretcherModule::stop()
{
    running_ = false;
    if (thread_.joinable()) thread_.join();

    {
        std::lock_guard<std::mutex> lock(motor_mtx_);
        for (auto& motor : motors_) motor.command_current = 0.f;
    }
    if (can_socket_ >= 0) send_current_commands();
    close_can();
}

void SpringStretcherModule::stretch()
{
    SpringStretcherCmd cmd;
    cmd.type = SpringStretcherCmd::Type::Stretch;
    cmd_buf_.write(cmd);
}

void SpringStretcherModule::retract()
{
    SpringStretcherCmd cmd;
    cmd.type = SpringStretcherCmd::Type::Retract;
    cmd_buf_.write(cmd);
}

void SpringStretcherModule::idle()
{
    SpringStretcherCmd cmd;
    cmd.type = SpringStretcherCmd::Type::Idle;
    cmd_buf_.write(cmd);
}

bool SpringStretcherModule::read_state(SpringStretcherState& out)
{
    return state_buf_.read(out);
}

bool SpringStretcherModule::open_can()
{
    can_socket_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (can_socket_ < 0) {
        std::fprintf(stderr, "[SpringStretcher] socket failed: %s\n", std::strerror(errno));
        return false;
    }

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, SPRING_STRETCHER_CAN_IFACE, IFNAMSIZ - 1);
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        std::fprintf(stderr, "[SpringStretcher] cannot find %s: %s\n",
                     SPRING_STRETCHER_CAN_IFACE, std::strerror(errno));
        close_can();
        return false;
    }

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::fprintf(stderr, "[SpringStretcher] bind %s failed: %s\n",
                     SPRING_STRETCHER_CAN_IFACE, std::strerror(errno));
        close_can();
        return false;
    }

    std::printf("[SpringStretcher] opened %s for two DJI M3508 motors\n",
                SPRING_STRETCHER_CAN_IFACE);
    return true;
}

void SpringStretcherModule::close_can()
{
    if (can_socket_ >= 0) {
        close(can_socket_);
        can_socket_ = -1;
    }
    hw_ok_ = false;
}

void SpringStretcherModule::loop()
{
    using Clock = std::chrono::steady_clock;
    SpringStretcherCmd cmd;

    while (running_) {
        auto t0 = Clock::now();

        while (read_feedback()) {}

        if (cmd_buf_.read(cmd)) execute_cmd(cmd);

        update_control();
        send_current_commands();
        publish_state();

        std::this_thread::sleep_until(t0 + LOOP_PERIOD);
    }
}

void SpringStretcherModule::execute_cmd(const SpringStretcherCmd& cmd)
{
    std::lock_guard<std::mutex> lock(motor_mtx_);

    switch (cmd.type) {
    case SpringStretcherCmd::Type::Idle:
        for (auto& motor : motors_) {
            motor.command_current = 0.f;
            motor.target = motor.position;
            motor.pos_i = 0.f;
            motor.vel_i = 0.f;
        }
        has_goal_ = false;
        retract_after_stretch_ = false;
        break;

    case SpringStretcherCmd::Type::Stretch:
        if (!calibrated()) {
            std::printf("[SpringStretcher] ignoring stretch command until calibration finishes\n");
            break;
        }
        for (auto& motor : motors_) {
            motor.target = motor.direction * SPRING_STRETCHER_STRETCH_RAD;
            motor.pos_i = 0.f;
            motor.vel_i = 0.f;
        }
        has_goal_ = true;
        retract_after_stretch_ = true;
        stretch_reached_at_ = {};
        std::printf("[SpringStretcher] stretching springs\n");
        break;

    case SpringStretcherCmd::Type::Retract:
        if (!calibrated()) {
            std::printf("[SpringStretcher] ignoring retract command until calibration finishes\n");
            break;
        }
        for (auto& motor : motors_) {
            motor.target = SPRING_STRETCHER_HOME_RAD;
            motor.pos_i = 0.f;
            motor.vel_i = 0.f;
        }
        has_goal_ = true;
        retract_after_stretch_ = false;
        std::printf("[SpringStretcher] retracting motors\n");
        break;
    }
}

bool SpringStretcherModule::read_feedback()
{
    if (can_socket_ < 0) return false;

    can_frame frame{};
    const auto n = read(can_socket_, &frame, sizeof(frame));
    if (n == static_cast<ssize_t>(sizeof(frame))) {
        process_feedback(frame);
        return true;
    }

    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        std::fprintf(stderr, "[SpringStretcher] CAN read failed: %s\n", std::strerror(errno));
    }
    return false;
}

void SpringStretcherModule::process_feedback(const can_frame& frame)
{
    std::lock_guard<std::mutex> lock(motor_mtx_);

    for (auto& motor : motors_) {
        if (frame.can_id != static_cast<canid_t>(0x200 + motor.id)) continue;

        const int16_t pos_raw =
            static_cast<int16_t>((static_cast<uint16_t>(frame.data[0]) << 8) |
                                 static_cast<uint16_t>(frame.data[1]));
        const int16_t vel_raw =
            static_cast<int16_t>((static_cast<uint16_t>(frame.data[2]) << 8) |
                                 static_cast<uint16_t>(frame.data[3]));
        const int16_t cur_raw =
            static_cast<int16_t>((static_cast<uint16_t>(frame.data[4]) << 8) |
                                 static_cast<uint16_t>(frame.data[5]));

        const float absolute_pos = static_cast<float>(pos_raw) * M3508_ENCODER_TO_RAD;
        if (!motor.raw_position_valid) {
            motor.raw_position = absolute_pos;
            motor.raw_position_valid = true;
        } else {
            motor.raw_position += wrap_delta(absolute_pos - motor.raw_position);
        }

        if (motor.zeroed) motor.position = motor.raw_position - motor.zero;
        else motor.position = 0.f;

        motor.velocity = static_cast<float>(vel_raw) * M3508_RPM_TO_RAD_S;
        motor.current = static_cast<float>(cur_raw) * M3508_CURRENT_SCALE_A;
        motor.feedback_ok = true;
    }
}

void SpringStretcherModule::update_control()
{
    using Clock = std::chrono::steady_clock;
    std::lock_guard<std::mutex> lock(motor_mtx_);

    const float dt = 1.0f / static_cast<float>(SPRING_STRETCHER_LOOP_HZ);
    const bool reached = has_goal_ && goal_reached();

    if (calibration_active_) {
        update_calibration();
        return;
    }

    if (retract_after_stretch_) {
        if (reached && stretch_reached_at_ == Clock::time_point{}) {
            stretch_reached_at_ = Clock::now();
        }
        if (stretch_reached_at_ != Clock::time_point{} &&
            std::chrono::duration<float>(Clock::now() - stretch_reached_at_).count() >=
                SPRING_STRETCHER_HOLD_TIME_S) {
            for (auto& motor : motors_) {
                motor.target = SPRING_STRETCHER_HOME_RAD;
                motor.pos_i = 0.f;
                motor.vel_i = 0.f;
            }
            retract_after_stretch_ = false;
            std::printf("[SpringStretcher] springs stretched, retracting motors\n");
        }
    }

    for (auto& motor : motors_) {
        if (!has_goal_ || !motor.feedback_ok) {
            motor.command_current = 0.f;
            continue;
        }

        const float prev_pos_error = motor.pos_error;
        motor.pos_error = motor.target - motor.position;
        motor.pos_i = clamp(motor.pos_i + motor.pos_error * dt, SPRING_STRETCHER_MAX_VEL_RAD_S);
        const float pos_d = (motor.pos_error - prev_pos_error) / dt;
        float goal_vel = SPRING_STRETCHER_POS_KP * motor.pos_error +
                         SPRING_STRETCHER_POS_KI * motor.pos_i +
                         SPRING_STRETCHER_POS_KD * pos_d;
        goal_vel = clamp(goal_vel, SPRING_STRETCHER_MAX_VEL_RAD_S);

        const float prev_vel_error = motor.vel_error;
        motor.vel_error = goal_vel - motor.velocity;
        motor.vel_i = clamp(motor.vel_i + motor.vel_error * dt, SPRING_STRETCHER_MAX_CURRENT_A);
        const float vel_d = (motor.vel_error - prev_vel_error) / dt;
        motor.command_current = SPRING_STRETCHER_VEL_KP * motor.vel_error +
                                SPRING_STRETCHER_VEL_KI * motor.vel_i +
                                SPRING_STRETCHER_VEL_KD * vel_d;
        motor.command_current = clamp(motor.command_current, SPRING_STRETCHER_MAX_CURRENT_A);
    }
}

void SpringStretcherModule::update_calibration()
{
    using Clock = std::chrono::steady_clock;
    const auto now = Clock::now();

    bool all_done = true;

    for (auto& motor : motors_) {
        if (motor.zeroed) {
            motor.command_current = 0.f;
            continue;
        }

        all_done = false;

        if (!motor.feedback_ok || !motor.raw_position_valid) {
            motor.command_current = 0.f;
            continue;
        }

        if (motor.calibration_started_at == Clock::time_point{}) {
            motor.calibration_started_at = now;
            motor.last_moving_at = now;
        }

        motor.command_current =
            clamp(-motor.direction * SPRING_STRETCHER_CALIB_CURRENT_A,
                  SPRING_STRETCHER_MAX_CURRENT_A);

        if (std::fabs(motor.velocity) > SPRING_STRETCHER_CALIB_JAM_VEL_RAD_S) {
            motor.last_moving_at = now;
        }

        const float still_time =
            std::chrono::duration<float>(now - motor.last_moving_at).count();
        const float elapsed =
            std::chrono::duration<float>(now - motor.calibration_started_at).count();

        if (still_time >= SPRING_STRETCHER_CALIB_JAM_TIME_S) {
            motor.zero = motor.raw_position;
            motor.position = 0.f;
            motor.target = SPRING_STRETCHER_HOME_RAD;
            motor.command_current = 0.f;
            motor.zeroed = true;
            std::printf("[SpringStretcher] motor %d top stop found\n", motor.id);
        } else if (elapsed >= SPRING_STRETCHER_CALIB_TIMEOUT_S) {
            for (auto& stop_motor : motors_) stop_motor.command_current = 0.f;
            motor.calibration_failed = true;
            hw_ok_ = false;
            calibration_active_ = false;
            std::fprintf(stderr, "[SpringStretcher] motor %d calibration timed out\n", motor.id);
            return;
        }
    }

    if (all_done || calibrated()) {
        calibration_active_ = false;
        has_goal_ = false;
        retract_after_stretch_ = false;
        std::printf("[SpringStretcher] calibration complete\n");
    }
}

void SpringStretcherModule::send_current_commands()
{
    if (can_socket_ < 0) return;

    can_frame frame_200{};
    can_frame frame_1ff{};
    frame_200.can_id = 0x200;
    frame_1ff.can_id = 0x1ff;
    frame_200.can_dlc = 8;
    frame_1ff.can_dlc = 8;

    {
        std::lock_guard<std::mutex> lock(motor_mtx_);
        for (const auto& motor : motors_) {
            const int16_t cmd =
                static_cast<int16_t>(clamp(motor.command_current, 20.0f) * M3508_CMD_SCALE);
            can_frame& frame = (motor.id <= 4) ? frame_200 : frame_1ff;
            const int slot = (motor.id <= 4) ? (motor.id - 1) : (motor.id - 5);
            frame.data[2 * slot] = static_cast<uint8_t>(cmd >> 8);
            frame.data[2 * slot + 1] = static_cast<uint8_t>(cmd & 0xff);
        }
    }

    if (write(can_socket_, &frame_200, sizeof(frame_200)) < 0 && errno != ENOBUFS) {
        std::fprintf(stderr, "[SpringStretcher] CAN write 0x200 failed: %s\n", std::strerror(errno));
    }
    if (SPRING_STRETCHER_LEFT_ID > 4 || SPRING_STRETCHER_RIGHT_ID > 4) {
        if (write(can_socket_, &frame_1ff, sizeof(frame_1ff)) < 0 && errno != ENOBUFS) {
            std::fprintf(stderr, "[SpringStretcher] CAN write 0x1ff failed: %s\n", std::strerror(errno));
        }
    }
}

void SpringStretcherModule::publish_state()
{
    SpringStretcherState state{};
    state.hw_ok = hw_ok_;

    {
        std::lock_guard<std::mutex> lock(motor_mtx_);
        for (std::size_t i = 0; i < motors_.size(); ++i) {
            state.position[i] = motors_[i].position;
            state.velocity[i] = motors_[i].velocity;
            state.current[i] = motors_[i].current;
        }
        state.calibrated = calibrated();
        state.reached_goal = has_goal_ && !retract_after_stretch_ && goal_reached();
    }

    state_buf_.write(state);
}

bool SpringStretcherModule::calibrated() const
{
    for (const auto& motor : motors_) {
        if (!motor.zeroed || motor.calibration_failed) return false;
    }
    return true;
}

bool SpringStretcherModule::goal_reached() const
{
    if (!calibrated()) return false;

    for (const auto& motor : motors_) {
        if (!motor.feedback_ok) return false;
        if (std::fabs(motor.target - motor.position) > SPRING_STRETCHER_POS_TOL_RAD) return false;
        if (std::fabs(motor.velocity) > SPRING_STRETCHER_VEL_TOL_RAD_S) return false;
    }
    return true;
}

float SpringStretcherModule::wrap_delta(float angle)
{
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

float SpringStretcherModule::clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}
