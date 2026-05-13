#include "config.hpp"
#include "camera.hpp"
#include "vision.hpp"
#include "arm.hpp"
#include "spring_stretcher.hpp"
#include "decision.hpp"
#include "runtime_config.hpp"
#include "log.hpp"

#include <algorithm>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <termios.h>
#include <unistd.h>

// ─────────────────────────────────────────────────────────────────────────────
static std::atomic<bool> g_shutdown{false};
static std::atomic<bool> g_keyboard_quit{false};

static void on_signal(int) {
    g_shutdown = true;
}

static void on_keyboard_signal(int) {
    g_keyboard_quit = true;
    g_shutdown = true;
}

class RawTerminalGuard {
public:
    RawTerminalGuard()
    {
        if (!isatty(STDIN_FILENO))
            return;
        if (tcgetattr(STDIN_FILENO, &old_term_) != 0)
            return;

        termios raw = old_term_;
        raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;

        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0)
            active_ = true;
    }

    ~RawTerminalGuard()
    {
        if (active_)
            tcsetattr(STDIN_FILENO, TCSANOW, &old_term_);
    }

    bool active() const { return active_; }

private:
    termios old_term_{};
    bool active_ = false;
};

static void keyboard_loop()
{
    using namespace std::chrono_literals;

    RawTerminalGuard terminal;
    if (!terminal.active()) {
        mdlog::warn("Input", "stdin is not a TTY; terminal q hotkey disabled");
        return;
    }

    while (!g_shutdown) {
        char ch = '\0';
        const ssize_t n = read(STDIN_FILENO, &ch, 1);
        if (n > 0 && (ch == 'q' || ch == 'Q')) {
            g_keyboard_quit = true;
            g_shutdown = true;
            break;
        }
        std::this_thread::sleep_for(30ms);
    }
}

static bool wait_for_arm_goal(ArmModule& arm, float timeout_s)
{
    using Clock = std::chrono::steady_clock;
    using namespace std::chrono_literals;

    const auto deadline = Clock::now() + std::chrono::duration<float>(timeout_s);
    ArmState state{};

    while (Clock::now() < deadline) {
        if (arm.read_state(state) && state.hw_ok && state.reached_goal)
            return true;
        std::this_thread::sleep_for(40ms);
    }
    return false;
}

static void emergency_return_home(ArmModule& arm, float duration_s)
{
    using namespace std::chrono_literals;

    if (duration_s < 0.3f)
        duration_s = 0.3f;

    mdlog::event("Main", "q pressed: stopping active trajectory and returning home");
    if (!arm.move_to("home", duration_s)) {
        mdlog::error("Main", "cannot return home: named position 'home' is unavailable");
        return;
    }

    const float timeout_s = std::max(2.5f, duration_s + 1.5f);
    if (wait_for_arm_goal(arm, timeout_s)) {
        mdlog::ok("Main", "arm reached home; disconnecting");
    } else {
        mdlog::warn("Main", "home return timed out after %.1fs; disconnecting anyway", timeout_s);
    }

    std::this_thread::sleep_for(100ms);
}

static bool starts_with_dash(const std::string& text)
{
    return !text.empty() && text[0] == '-';
}

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);
    std::signal(SIGUSR1, on_keyboard_signal);

    mdlog::event("Main", "Meta-Dart dart launcher starting");

    RuntimeConfig runtime_config = make_default_runtime_config();
    const char* config_path = "config/loader_config.json";
    bool explicit_config = false;
    bool check_config_only = false;
    std::string pickup_mode_override;

    if (const char* env = std::getenv("META_DART_CONFIG")) {
        config_path = env;
        explicit_config = true;
    }

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--check-config") {
            check_config_only = true;
            explicit_config = true;
        } else if (arg == "--mode" || arg == "--pickup-mode") {
            if (i + 1 >= argc) {
                mdlog::error("Main", "%s requires 'disturb' or 'replay'", arg.c_str());
                return 1;
            }
            pickup_mode_override = argv[++i];
        } else if (arg == "--replay") {
            pickup_mode_override = "replay";
        } else if (arg == "--disturb" || arg == "--allow-disturb") {
            pickup_mode_override = "disturb";
        } else if (!starts_with_dash(arg)) {
            config_path = argv[i];
            explicit_config = true;
        } else {
            mdlog::error("Main", "unknown argument: %s", arg.c_str());
            return 1;
        }
    }

    std::string config_error;
    if (load_runtime_config_json(config_path, runtime_config, config_error)) {
        mdlog::ok("Main", "loaded runtime config: %s", config_path);
    } else if (explicit_config) {
        mdlog::error("Main", "failed to load runtime config '%s': %s",
                     config_path, config_error.c_str());
        return 1;
    } else {
        mdlog::warn("Main", "using built-in defaults; config '%s' not loaded: %s",
                    config_path, config_error.c_str());
    }

    if (!pickup_mode_override.empty()) {
        std::string mode_error;
        if (!set_pickup_mode(runtime_config.decision, pickup_mode_override, mode_error)) {
            mdlog::error("Main", "%s", mode_error.c_str());
            return 1;
        }
        mdlog::system("pickup mode overridden: %s", runtime_config.decision.pickup_mode.c_str());
    }

    if (check_config_only) {
        mdlog::ok("Main", "config OK");
        mdlog::info("Config", "vision confidence threshold: %.2f",
                    runtime_config.vision_confidence_threshold);
        mdlog::info("Config", "zones=%zu trajectories=%zu reset=%.1fs stable=%.1fs search_grace=%.2fs search_margin=%.3fm",
                    runtime_config.decision.zones.size(),
                    runtime_config.decision.trajectories.size(),
                    runtime_config.decision.reset_cooldown_s,
                    runtime_config.decision.stable_dart_time_s,
                    runtime_config.decision.search_stability_grace_s,
                    runtime_config.decision.search_stability_bbox_margin_m);
        mdlog::info("Config", "pickup_mode=%s motion_monitor=%s fast_speedup=%.2fx emergency_home=%.2fs pickup_margin=%.3fm hold=%.1fs relocate_window=%.1fs relocate_stable=%.1fs grace=%.2fs margin=%.3fm same_slot=%s",
                    runtime_config.decision.pickup_mode.c_str(),
                    runtime_config.decision.monitor_dart_motion_during_pickup ? "true" : "false",
                    runtime_config.decision.fast_motion_speedup,
                    runtime_config.decision.emergency_home_duration_s,
                    runtime_config.decision.pickup_motion_bbox_margin_m,
                    runtime_config.decision.dart_motion_hold_s,
                    runtime_config.decision.relocation_window_s,
                    runtime_config.decision.relocation_stable_time_s,
                    runtime_config.decision.relocation_outside_grace_s,
                    runtime_config.decision.relocation_bbox_margin_m,
                    runtime_config.decision.relocation_accept_same_slot ? "true" : "false");
        return 0;
    }

    // ── Instantiate modules ───────────────────────────────────────────────
    CameraModule   camera(CAMERA_INDEX);
    VisionModule   vision(camera.buffer(), runtime_config.vision_confidence_threshold);
    ArmModule      arm;
    SpringStretcherModule spring_stretcher;
    DecisionModule        decision(vision.buffer(), arm, spring_stretcher, runtime_config.decision);

    // ── Start modules ────────────────────────────────────────────────────
    if (!camera.start()) {
        mdlog::error("Main", "camera failed to start");
        return 1;
    }

    vision.start();

    const bool arm_online = arm.start();
    if (!arm_online) {
        mdlog::warn("Main", "arm failed to start; running in camera-only mode");
        // Continue without the arm so vision can still be tested
    }

    if (!spring_stretcher.start())
        mdlog::warn("Main", "spring stretcher failed to start; shooting disabled");

    decision.start();

    std::thread keyboard_thread(keyboard_loop);

    mdlog::event("Main", "running; press q to stop, return home, and disconnect; Ctrl-C stops normally");

    // ── Spin until SIGINT ─────────────────────────────────────────────────
    while (!g_shutdown) {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(100ms);
    }

    // ── Shutdown in reverse order ─────────────────────────────────────────
    mdlog::event("Main", "shutdown requested");
    decision.stop();
    if (arm_online && g_keyboard_quit)
        emergency_return_home(arm, runtime_config.decision.emergency_home_duration_s);
    spring_stretcher.stop();
    arm.stop();
    vision.stop();
    camera.stop();

    if (keyboard_thread.joinable())
        keyboard_thread.join();

    mdlog::ok("Main", "done");
    return 0;
}
