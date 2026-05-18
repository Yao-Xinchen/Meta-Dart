#pragma once

#include <chrono>
#include <filesystem>
#include <string>

namespace mg995 {

constexpr int PERIOD_NS = 20'000'000;
constexpr const char* PIN18_PWM_DEVICE = "fd8b0000.pwm";
constexpr int MIN_PULSE_US = 1000;
constexpr int MAX_PULSE_US = 2000;
constexpr float MIN_ANGLE_DEG = 20.0f;
constexpr float MAX_ANGLE_DEG = 160.0f;
constexpr float DEFAULT_REST_DEG = 90.0f;
constexpr float DEFAULT_STEP_DEG = 2.0f;
constexpr auto DEFAULT_STEP_DELAY = std::chrono::milliseconds(20);
constexpr auto DEFAULT_SETTLE_TIME = std::chrono::milliseconds(250);
constexpr auto MAX_ENABLED_TIME = std::chrono::seconds(3);

bool write_text(const std::filesystem::path& path, const std::string& value);
float clamp_angle(float deg);
int angle_to_pulse_us(float deg);
std::filesystem::path find_pin18_pwmchip();
std::filesystem::path resolve_pwmchip(const std::string& chip);

class SysfsPwm {
public:
    SysfsPwm(std::filesystem::path chip, int channel);
    ~SysfsPwm();

    bool open();
    bool set_angle(float deg);
    bool move_slow(float from_deg,
                   float to_deg,
                   float step_deg = DEFAULT_STEP_DEG,
                   std::chrono::milliseconds step_delay = DEFAULT_STEP_DELAY);
    void disable();

private:
    bool enabled_time_exceeded() const;

    std::filesystem::path chip_;
    int channel_ = 0;
    std::filesystem::path pwm_;
    bool enabled_ = false;
    std::chrono::steady_clock::time_point enabled_since_{};
};

}  // namespace mg995
