#pragma once

#include "config.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

namespace stretcher_test {

static constexpr float PI = 3.14159265358979323846f;
static constexpr float M3508_ENCODER_TO_RAD = 2.0f * PI / 8192.0f;
static constexpr float M3508_RPM_TO_RAD_S = PI / 30.0f;
static constexpr float M3508_CURRENT_SCALE_A = 20.0f / 16384.0f;
static constexpr float M3508_CMD_SCALE = 16384.0f / 20.0f;

inline float clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

inline float wrap_delta(float angle)
{
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

struct Feedback {
    int id = 1;
    bool seen = false;
    bool raw_valid = false;
    float raw_position = 0.f; // cumulative encoder angle since program start [rad]
    float velocity = 0.f;     // feedback velocity [rad/s]
    float current = 0.f;      // feedback current [A]
};

class M3508Bus {
public:
    bool open(const char* iface)
    {
        fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
        if (fd_ < 0) {
            std::fprintf(stderr, "socket failed: %s\n", std::strerror(errno));
            return false;
        }

        ifreq ifr{};
        std::strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
        if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
            std::fprintf(stderr, "cannot find %s: %s\n", iface, std::strerror(errno));
            close();
            return false;
        }

        sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            std::fprintf(stderr, "bind %s failed: %s\n", iface, std::strerror(errno));
            close();
            return false;
        }

        return true;
    }

    void close()
    {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    ~M3508Bus()
    {
        close();
    }

    bool poll(std::array<Feedback, 2>& fb)
    {
        bool got = false;
        can_frame frame{};
        while (fd_ >= 0) {
            const auto n = read(fd_, &frame, sizeof(frame));
            if (n == static_cast<ssize_t>(sizeof(frame))) {
                process_frame(frame, fb);
                got = true;
                continue;
            }
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
                std::fprintf(stderr, "CAN read failed: %s\n", std::strerror(errno));
            break;
        }
        return got;
    }

    void send(float left_current, float right_current, int left_id, int right_id)
    {
        can_frame frame_200{};
        can_frame frame_1ff{};
        frame_200.can_id = 0x200;
        frame_1ff.can_id = 0x1ff;
        frame_200.can_dlc = 8;
        frame_1ff.can_dlc = 8;

        put_current(frame_200, frame_1ff, left_id, left_current);
        put_current(frame_200, frame_1ff, right_id, right_current);

        if (write(fd_, &frame_200, sizeof(frame_200)) < 0 && errno != ENOBUFS)
            std::fprintf(stderr, "CAN write 0x200 failed: %s\n", std::strerror(errno));

        if (left_id > 4 || right_id > 4) {
            if (write(fd_, &frame_1ff, sizeof(frame_1ff)) < 0 && errno != ENOBUFS)
                std::fprintf(stderr, "CAN write 0x1ff failed: %s\n", std::strerror(errno));
        }
    }

private:
    int fd_ = -1;

    static void process_frame(const can_frame& frame, std::array<Feedback, 2>& fb)
    {
        for (auto& motor : fb) {
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

            const float pos = static_cast<float>(pos_raw) * M3508_ENCODER_TO_RAD;
            if (!motor.raw_valid) {
                motor.raw_position = pos;
                motor.raw_valid = true;
            } else {
                motor.raw_position += wrap_delta(pos - motor.raw_position);
            }

            motor.velocity = static_cast<float>(vel_raw) * M3508_RPM_TO_RAD_S;
            motor.current = static_cast<float>(cur_raw) * M3508_CURRENT_SCALE_A;
            motor.seen = true;
        }
    }

    static void put_current(can_frame& frame_200, can_frame& frame_1ff, int id, float current)
    {
        const int16_t cmd = static_cast<int16_t>(clamp(current, 20.0f) * M3508_CMD_SCALE);
        can_frame& frame = (id <= 4) ? frame_200 : frame_1ff;
        const int slot = (id <= 4) ? (id - 1) : (id - 5);
        frame.data[2 * slot] = static_cast<uint8_t>(cmd >> 8);
        frame.data[2 * slot + 1] = static_cast<uint8_t>(cmd & 0xff);
    }
};

struct CalibResult {
    bool ok = false;
    std::array<float, 2> zero = {};
    std::array<float, 2> travel_to_top = {};
};

inline void print_feedback(const std::array<Feedback, 2>& fb)
{
    // pos is cumulative raw motor angle, not calibrated hook length.
    // seen=0 means no matching feedback frame has arrived for that motor ID.
    std::printf("left: pos=%8.3f vel=%8.3f cur=%6.2f seen=%d | "
                "right: pos=%8.3f vel=%8.3f cur=%6.2f seen=%d\n",
                fb[0].raw_position, fb[0].velocity, fb[0].current, static_cast<int>(fb[0].seen),
                fb[1].raw_position, fb[1].velocity, fb[1].current, static_cast<int>(fb[1].seen));
}

inline CalibResult calibrate_top_stops(M3508Bus& bus,
                                       std::array<Feedback, 2>& fb,
                                       float current_a,
                                       float jam_vel,
                                       float jam_time_s,
                                       float timeout_s,
                                       float left_dir,
                                       float right_dir)
{
    using Clock = std::chrono::steady_clock;

    CalibResult result{};
    std::array<bool, 2> done = {false, false};
    std::array<float, 2> start_pos = {};
    std::array<Clock::time_point, 2> started = {};
    std::array<Clock::time_point, 2> last_moving = {};

    const auto t0 = Clock::now();
    const auto print_period = std::chrono::milliseconds(100);
    auto next_print = t0;

    while (true) {
        const auto now = Clock::now();
        bus.poll(fb);

        float left_current = done[0] ? 0.f : -left_dir * current_a;
        float right_current = done[1] ? 0.f : -right_dir * current_a;

        for (int i = 0; i < 2; ++i) {
            if (done[i] || !fb[i].seen) continue;
            if (started[i] == Clock::time_point{}) {
                started[i] = now;
                last_moving[i] = now;
                start_pos[i] = fb[i].raw_position;
            }

            if (std::fabs(fb[i].velocity) > jam_vel)
                last_moving[i] = now;

            const float still_s = std::chrono::duration<float>(now - last_moving[i]).count();
            if (still_s >= jam_time_s) {
                done[i] = true;
                result.zero[i] = fb[i].raw_position;
                result.travel_to_top[i] = fb[i].raw_position - start_pos[i];
                std::printf("motor %d top stop: zero=%.3f travel=%.3f rad\n",
                            fb[i].id, result.zero[i], result.travel_to_top[i]);
            }
        }

        bus.send(left_current, right_current, fb[0].id, fb[1].id);

        if (now >= next_print) {
            print_feedback(fb);
            next_print = now + print_period;
        }

        if (done[0] && done[1]) {
            result.ok = true;
            break;
        }

        if (std::chrono::duration<float>(now - t0).count() >= timeout_s) {
            std::fprintf(stderr, "calibration timed out\n");
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    bus.send(0.f, 0.f, fb[0].id, fb[1].id);
    return result;
}

}  // namespace stretcher_test
