#include "mg995_servo.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>

#include <fcntl.h>
#include <unistd.h>

namespace mg995 {

bool write_text(const std::filesystem::path& path, const std::string& value)
{
    const int fd = open(path.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        std::fprintf(stderr, "ERROR: cannot open %s: %s\n",
                     path.c_str(), std::strerror(errno));
        return false;
    }

    const std::string text = value + "\n";
    const ssize_t n = write(fd, text.data(), text.size());
    if (n != static_cast<ssize_t>(text.size())) {
        const int saved_errno = errno;
        close(fd);
        std::fprintf(stderr, "ERROR: cannot write %s: %s\n",
                     path.c_str(), std::strerror(saved_errno));
        return false;
    }

    if (close(fd) != 0) {
        std::fprintf(stderr, "ERROR: cannot close %s after write: %s\n",
                     path.c_str(), std::strerror(errno));
        return false;
    }
    return true;
}

float clamp_angle(float deg)
{
    return std::clamp(deg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
}

int angle_to_pulse_us(float deg)
{
    const float safe_deg = clamp_angle(deg);
    const float t = (safe_deg - MIN_ANGLE_DEG) / (MAX_ANGLE_DEG - MIN_ANGLE_DEG);
    return static_cast<int>(std::lround(MIN_PULSE_US + t * (MAX_PULSE_US - MIN_PULSE_US)));
}

std::filesystem::path find_pin18_pwmchip()
{
    const std::filesystem::path pwm_class = "/sys/class/pwm";
    std::error_code ec;
    if (!std::filesystem::exists(pwm_class, ec))
        return {};

    for (const auto& entry : std::filesystem::directory_iterator(pwm_class, ec)) {
        if (ec) break;
        if (entry.path().filename().string().rfind("pwmchip", 0) != 0)
            continue;

        const auto resolved = std::filesystem::canonical(entry.path(), ec);
        if (ec) {
            ec.clear();
            continue;
        }

        if (resolved.string().find(PIN18_PWM_DEVICE) != std::string::npos)
            return entry.path();
    }

    return {};
}

std::filesystem::path resolve_pwmchip(const std::string& chip)
{
    if (chip == "auto" || chip == "pin18" || chip == "pwm0-m1")
        return find_pin18_pwmchip();
    return chip;
}

SysfsPwm::SysfsPwm(std::filesystem::path chip, int channel)
    : chip_(std::move(chip))
    , channel_(channel)
    , pwm_(chip_ / ("pwm" + std::to_string(channel_)))
{
}

SysfsPwm::~SysfsPwm()
{
    disable();
}

bool SysfsPwm::open()
{
    if (!std::filesystem::exists(chip_)) {
        std::fprintf(stderr, "ERROR: PWM chip does not exist: %s\n", chip_.c_str());
        return false;
    }

    std::error_code ec;
    const auto resolved = std::filesystem::canonical(chip_, ec);
    if (!ec)
        std::printf("Resolved PWM chip: %s\n", resolved.c_str());

    if (!std::filesystem::exists(pwm_)) {
        if (!write_text(chip_ / "export", std::to_string(channel_))) {
            std::fprintf(stderr,
                         "Hint: if the error is 'Device or resource busy', this PWM is\n"
                         "      already owned by a kernel driver. Pick another PWM overlay/pin.\n"
                         "      Otherwise, run with sudo and check the channel number.\n");
            return false;
        }

        for (int i = 0; i < 20 && !std::filesystem::exists(pwm_); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!std::filesystem::exists(pwm_)) {
        std::fprintf(stderr, "ERROR: PWM channel was not created: %s\n", pwm_.c_str());
        return false;
    }

    // period and duty_cycle must be set before enable/polarity can be written
    if (!write_text(pwm_ / "period", std::to_string(PERIOD_NS)))
        return false;
    if (!write_text(pwm_ / "duty_cycle", "0"))
        return false;
    if (std::filesystem::exists(pwm_ / "polarity")) {
        if (!write_text(pwm_ / "polarity", "normal"))
            return false;
    }
    disable();

    return true;
}

bool SysfsPwm::set_angle(float deg)
{
    const int pulse_us = angle_to_pulse_us(deg);
    const int duty_ns = pulse_us * 1000;
    if (!write_text(pwm_ / "duty_cycle", std::to_string(duty_ns)))
        return false;
    if (!enabled_) {
        if (!write_text(pwm_ / "enable", "1"))
            return false;
        enabled_ = true;
        enabled_since_ = std::chrono::steady_clock::now();
    }
    std::printf("angle %.1f deg -> pulse %d us\n", clamp_angle(deg), pulse_us);
    return true;
}

bool SysfsPwm::move_slow(float from_deg,
                         float to_deg,
                         float step_deg,
                         std::chrono::milliseconds step_delay)
{
    float current = clamp_angle(from_deg);
    const float target = clamp_angle(to_deg);
    const float direction = (target >= current) ? 1.0f : -1.0f;
    const float step = std::max(0.1f, std::fabs(step_deg));

    while (std::fabs(target - current) > 0.01f) {
        if (enabled_time_exceeded()) {
            std::fprintf(stderr, "ERROR: PWM enabled too long; disabling for protection\n");
            disable();
            return false;
        }

        if (!set_angle(current))
            return false;
        if (std::fabs(target - current) <= step) {
            current = target;
        } else {
            current += direction * step;
        }
        std::this_thread::sleep_for(step_delay);
    }

    return set_angle(target);
}

void SysfsPwm::disable()
{
    if (std::filesystem::exists(pwm_ / "enable"))
        write_text(pwm_ / "enable", "0");
    enabled_ = false;
}

bool SysfsPwm::enabled_time_exceeded() const
{
    return enabled_ && (std::chrono::steady_clock::now() - enabled_since_ > MAX_ENABLED_TIME);
}

}  // namespace mg995
