// test/test_mg995_servo.cpp
//
// Standalone MG995 servo test over Linux sysfs PWM. This uses the same
// mg995::SysfsPwm helper as TriggerModule, so tested angles map directly to
// the launcher trigger configuration in include/config.hpp.
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

#include "config.hpp"
#include "mg995_servo.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace {

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

void print_usage(const char* argv0)
{
    std::printf("Usage:\n");
    std::printf("  sudo %s [pwmchip] [channel] [command] [args]\n\n", argv0);
    std::printf("Defaults:\n");
    std::printf("  pwmchip=auto, finds %s / PWM0_M1 / 26-pin pin 18\n",
                mg995::PIN18_PWM_DEVICE);
    std::printf("  channel=0  command=center\n\n");
    std::printf("Commands:\n");
    std::printf("  center\n");
    std::printf("  hold                  # move to TRIGGER_HOLD_POS_DEG\n");
    std::printf("  release               # move to TRIGGER_RELEASE_POS_DEG\n");
    std::printf("  angle <deg>\n");
    std::printf("  sweep <min_deg> <max_deg>\n");
    std::printf("  fire <rest_deg> <pull_deg> <hold_ms>\n\n");
    std::printf("Trigger config:\n");
    std::printf("  hold=%.1f deg release=%.1f deg settle=%.0f ms release_pulse=%.0f ms\n",
                TRIGGER_HOLD_POS_DEG,
                TRIGGER_RELEASE_POS_DEG,
                TRIGGER_SETTLE_TIME_S * 1000.0f,
                TRIGGER_RELEASE_PULSE_S * 1000.0f);
    std::printf("Software protection:\n");
    std::printf("  angle clamp %.0f..%.0f deg, pulse clamp %d..%d us, auto detach after motion\n",
                mg995::MIN_ANGLE_DEG,
                mg995::MAX_ANGLE_DEG,
                mg995::MIN_PULSE_US,
                mg995::MAX_PULSE_US);
}

}  // namespace

int main(int argc, char* argv[])
{
    std::string chip_arg = "auto";
    int channel = 0;
    std::string command = "center";
    int argi = 1;

    if (argc > argi && std::string(argv[argi]) == "--help") {
        print_usage(argv[0]);
        return 0;
    }
    if (argc > argi) chip_arg = argv[argi++];
    if (argc > argi) channel = parse_int(argv[argi++], 0);
    if (argc > argi) command = argv[argi++];

    const std::filesystem::path chip = mg995::resolve_pwmchip(chip_arg);
    if (chip.empty()) {
        std::fprintf(stderr,
                     "ERROR: cannot find %s under /sys/class/pwm.\n"
                     "Enable the Orange Pi overlay for 26-pin pin 18 first:\n"
                     "  overlays=pwm0-m1\n"
                     "in /boot/orangepiEnv.txt, then reboot.\n",
                     mg995::PIN18_PWM_DEVICE);
        return 1;
    }

    std::printf("=== MG995 sysfs PWM test ===\n");
    std::printf("PWM: %s pwm%d\n", chip.c_str(), channel);
    std::printf("IMPORTANT: use external 5-6 V servo power and common ground.\n");

    mg995::SysfsPwm pwm(chip, channel);
    if (!pwm.open())
        return 1;

    bool ok = true;
    if (command == "center") {
        ok = pwm.set_angle(mg995::DEFAULT_REST_DEG);
        std::this_thread::sleep_for(mg995::DEFAULT_SETTLE_TIME);
    } else if (command == "hold") {
        ok = pwm.move_slow(mg995::DEFAULT_REST_DEG, TRIGGER_HOLD_POS_DEG);
        std::this_thread::sleep_for(mg995::DEFAULT_SETTLE_TIME);
    } else if (command == "release") {
        ok = pwm.move_slow(mg995::DEFAULT_REST_DEG, TRIGGER_RELEASE_POS_DEG);
        std::this_thread::sleep_for(mg995::DEFAULT_SETTLE_TIME);
    } else if (command == "angle") {
        const float deg = (argc > argi) ? parse_float(argv[argi], mg995::DEFAULT_REST_DEG)
                                        : mg995::DEFAULT_REST_DEG;
        ok = pwm.move_slow(mg995::DEFAULT_REST_DEG, deg);
        std::this_thread::sleep_for(mg995::DEFAULT_SETTLE_TIME);
    } else if (command == "sweep") {
        const float min_deg = (argc > argi) ? parse_float(argv[argi++], 60.0f) : 60.0f;
        const float max_deg = (argc > argi) ? parse_float(argv[argi++], 120.0f) : 120.0f;
        ok = pwm.move_slow(mg995::DEFAULT_REST_DEG, min_deg);
        if (ok) ok = pwm.move_slow(min_deg, max_deg);
        if (ok) ok = pwm.move_slow(max_deg, mg995::DEFAULT_REST_DEG);
        std::this_thread::sleep_for(mg995::DEFAULT_SETTLE_TIME);
    } else if (command == "fire") {
        const float rest_deg = (argc > argi) ? parse_float(argv[argi++], TRIGGER_HOLD_POS_DEG)
                                             : TRIGGER_HOLD_POS_DEG;
        const float pull_deg = (argc > argi) ? parse_float(argv[argi++], TRIGGER_RELEASE_POS_DEG)
                                             : TRIGGER_RELEASE_POS_DEG;
        const int hold_ms = std::clamp((argc > argi) ? parse_int(argv[argi++], 300) : 300,
                                       50,
                                       1000);
        ok = pwm.move_slow(rest_deg, pull_deg);
        std::this_thread::sleep_for(std::chrono::milliseconds(hold_ms));
        if (ok) ok = pwm.move_slow(pull_deg, rest_deg);
        std::this_thread::sleep_for(mg995::DEFAULT_SETTLE_TIME);
    } else {
        std::fprintf(stderr, "ERROR: unknown command: %s\n", command.c_str());
        print_usage(argv[0]);
        ok = false;
    }

    pwm.disable();
    std::printf("PWM disabled. Test %s.\n", ok ? "complete" : "failed");
    return ok ? 0 : 1;
}
