// test/test_mg995_servo.cpp
//
// Standalone MG995 servo test over Linux sysfs PWM.
//
// Build:
//   cmake --build build --target test_mg995_servo
//
// Examples:
//   sudo ./build/test_mg995_servo
//   sudo ./build/test_mg995_servo auto 0 center
//   sudo ./build/test_mg995_servo auto 0 angle 90
//   sudo ./build/test_mg995_servo auto 0 sweep 60 120
//   sudo ./build/test_mg995_servo auto 0 fire 90 45 300
//
// Default PWM:
//   auto-detect the sysfs pwmchip that resolves to fd8b0000.pwm. On Orange Pi 5
//   this is PWM0_M1 on 26-pin physical pin 18.
//
// Hardware note:
//   Power MG995 from a separate 5-6 V supply. Connect servo GND, supply GND,
//   and Orange Pi GND together. Do not power MG995 from the Orange Pi 5V pin.

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <thread>

#include <fcntl.h>
#include <unistd.h>

using namespace std::chrono_literals;

namespace {

// Conservative MG995 limits. Wider ranges such as 500-2500 us may hit hard
// stops on some linkages, which causes high current.
constexpr int kPeriodNs = 20'000'000;       // 50 Hz
constexpr const char* kPin18PwmDevice = "fd8b0000.pwm";
constexpr int kMinPulseUs = 1000;           // protected lower pulse
constexpr int kMaxPulseUs = 2000;           // protected upper pulse
constexpr float kMinAngleDeg = 20.0f;       // software travel limit
constexpr float kMaxAngleDeg = 160.0f;      // software travel limit
constexpr float kDefaultRestDeg = 90.0f;
constexpr float kStepDeg = 2.0f;            // slow movement reduces current spikes
constexpr auto kStepDelay = 20ms;
constexpr auto kSettleTime = 250ms;
constexpr auto kMaxEnabledTime = 3s;        // detach quickly to avoid stall heating

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
    return std::clamp(deg, kMinAngleDeg, kMaxAngleDeg);
}

int angle_to_pulse_us(float deg)
{
    const float safe_deg = clamp_angle(deg);
    const float t = (safe_deg - kMinAngleDeg) / (kMaxAngleDeg - kMinAngleDeg);
    return static_cast<int>(std::lround(kMinPulseUs + t * (kMaxPulseUs - kMinPulseUs)));
}

class SysfsPwm {
public:
    SysfsPwm(std::filesystem::path chip, int channel)
        : chip_(std::move(chip))
        , channel_(channel)
        , pwm_(chip_ / ("pwm" + std::to_string(channel_)))
    {
    }

    ~SysfsPwm()
    {
        disable();
    }

    bool open()
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
                std::this_thread::sleep_for(10ms);
        }

        if (!std::filesystem::exists(pwm_)) {
            std::fprintf(stderr, "ERROR: PWM channel was not created: %s\n", pwm_.c_str());
            return false;
        }

        // Disable before changing period to avoid kernel EBUSY on some drivers.
        disable();
        if (std::filesystem::exists(pwm_ / "polarity")) {
            if (!write_text(pwm_ / "polarity", "normal"))
                return false;
        }
        if (!write_text(pwm_ / "period", std::to_string(kPeriodNs)))
            return false;
        if (!write_text(pwm_ / "duty_cycle", "0"))
            return false;

        return true;
    }

    bool set_angle(float deg)
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

    bool move_slow(float from_deg, float to_deg)
    {
        float current = clamp_angle(from_deg);
        const float target = clamp_angle(to_deg);
        const float direction = (target >= current) ? 1.0f : -1.0f;

        while (std::fabs(target - current) > 0.01f) {
            if (enabled_time_exceeded()) {
                std::fprintf(stderr, "ERROR: PWM enabled too long; disabling for protection\n");
                disable();
                return false;
            }

            if (!set_angle(current))
                return false;
            if (std::fabs(target - current) <= kStepDeg) {
                current = target;
            } else {
                current += direction * kStepDeg;
            }
            std::this_thread::sleep_for(kStepDelay);
        }

        return set_angle(target);
    }

    void disable()
    {
        if (std::filesystem::exists(pwm_ / "enable"))
            write_text(pwm_ / "enable", "0");
        enabled_ = false;
    }

private:
    bool enabled_time_exceeded() const
    {
        return enabled_ && (std::chrono::steady_clock::now() - enabled_since_ > kMaxEnabledTime);
    }

    std::filesystem::path chip_;
    int channel_ = 0;
    std::filesystem::path pwm_;
    bool enabled_ = false;
    std::chrono::steady_clock::time_point enabled_since_{};
};

float parse_float(const char* text, float fallback)
{
    if (!text) return fallback;
    char* end = nullptr;
    const float value = std::strtof(text, &end);
    return (end && *end == '\0') ? value : fallback;
}

int parse_int(const char* text, int fallback)
{
    if (!text) return fallback;
    char* end = nullptr;
    const long value = std::strtol(text, &end, 10);
    return (end && *end == '\0') ? static_cast<int>(value) : fallback;
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

        if (resolved.string().find(kPin18PwmDevice) != std::string::npos)
            return entry.path();
    }

    return {};
}

void print_usage(const char* argv0)
{
    std::printf("Usage:\n");
    std::printf("  sudo %s [pwmchip] [channel] [command] [args]\n\n", argv0);
    std::printf("Defaults:\n");
    std::printf("  pwmchip=auto, finds %s / PWM0_M1 / 26-pin pin 18\n", kPin18PwmDevice);
    std::printf("  channel=0  command=center\n\n");
    std::printf("Commands:\n");
    std::printf("  center\n");
    std::printf("  angle <deg>\n");
    std::printf("  sweep <min_deg> <max_deg>\n");
    std::printf("  fire <rest_deg> <pull_deg> <hold_ms>\n\n");
    std::printf("Software protection:\n");
    std::printf("  angle clamp %.0f..%.0f deg, pulse clamp %d..%d us, auto detach after motion\n",
                kMinAngleDeg, kMaxAngleDeg, kMinPulseUs, kMaxPulseUs);
}

}  // namespace

int main(int argc, char* argv[])
{
    std::filesystem::path chip = "auto";
    int channel = 0;
    std::string command = "center";
    int argi = 1;

    if (argc > argi && std::string(argv[argi]) == "--help") {
        print_usage(argv[0]);
        return 0;
    }
    if (argc > argi) chip = argv[argi++];
    if (argc > argi) channel = parse_int(argv[argi++], 0);
    if (argc > argi) command = argv[argi++];

    if (chip == "auto" || chip == "pin18" || chip == "pwm0-m1") {
        chip = find_pin18_pwmchip();
        if (chip.empty()) {
            std::fprintf(stderr,
                         "ERROR: cannot find %s under /sys/class/pwm.\n"
                         "Enable the Orange Pi overlay for 26-pin pin 18 first:\n"
                         "  overlays=pwm0-m1\n"
                         "in /boot/orangepiEnv.txt, then reboot.\n",
                         kPin18PwmDevice);
            return 1;
        }
    }

    std::printf("=== MG995 sysfs PWM test ===\n");
    std::printf("PWM: %s pwm%d\n", chip.c_str(), channel);
    std::printf("IMPORTANT: use external 5-6 V servo power and common ground.\n");

    SysfsPwm pwm(chip, channel);
    if (!pwm.open())
        return 1;

    bool ok = true;
    if (command == "center") {
        ok = pwm.set_angle(kDefaultRestDeg);
        std::this_thread::sleep_for(kSettleTime);
    } else if (command == "angle") {
        const float deg = (argc > argi) ? parse_float(argv[argi], kDefaultRestDeg) : kDefaultRestDeg;
        ok = pwm.move_slow(kDefaultRestDeg, deg);
        std::this_thread::sleep_for(kSettleTime);
    } else if (command == "sweep") {
        const float min_deg = (argc > argi) ? parse_float(argv[argi++], 60.0f) : 60.0f;
        const float max_deg = (argc > argi) ? parse_float(argv[argi++], 120.0f) : 120.0f;
        ok = pwm.move_slow(kDefaultRestDeg, min_deg);
        if (ok) ok = pwm.move_slow(min_deg, max_deg);
        if (ok) ok = pwm.move_slow(max_deg, kDefaultRestDeg);
        std::this_thread::sleep_for(kSettleTime);
    } else if (command == "fire") {
        const float rest_deg = (argc > argi) ? parse_float(argv[argi++], 90.0f) : 90.0f;
        const float pull_deg = (argc > argi) ? parse_float(argv[argi++], 45.0f) : 45.0f;
        const int hold_ms = std::clamp((argc > argi) ? parse_int(argv[argi++], 300) : 300, 50, 1000);
        ok = pwm.move_slow(rest_deg, pull_deg);
        std::this_thread::sleep_for(std::chrono::milliseconds(hold_ms));
        if (ok) ok = pwm.move_slow(pull_deg, rest_deg);
        std::this_thread::sleep_for(kSettleTime);
    } else {
        std::fprintf(stderr, "ERROR: unknown command: %s\n", command.c_str());
        print_usage(argv[0]);
        ok = false;
    }

    pwm.disable();
    std::printf("PWM disabled. Test %s.\n", ok ? "complete" : "failed");
    return ok ? 0 : 1;
}
