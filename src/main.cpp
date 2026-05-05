#include "config.hpp"
#include "camera.hpp"
#include "vision.hpp"
#include "arm.hpp"
#include "decision.hpp"
#include "runtime_config.hpp"

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <atomic>
#include <string>

// ─────────────────────────────────────────────────────────────────────────────
static std::atomic<bool> g_shutdown{false};

static void on_signal(int) {
    g_shutdown = true;
}

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    std::printf("[Main] Starting loader\n");

    RuntimeConfig runtime_config = make_default_runtime_config();
    const char* config_path = "config/loader_config.json";
    bool explicit_config = false;
    bool check_config_only = false;

    if (const char* env = std::getenv("META_DART_CONFIG")) {
        config_path = env;
        explicit_config = true;
    }
    if (argc > 1 && std::string(argv[1]) == "--check-config") {
        check_config_only = true;
        explicit_config = true;
        if (argc > 2)
            config_path = argv[2];
    } else if (argc > 1) {
        config_path = argv[1];
        explicit_config = true;
    }

    std::string config_error;
    if (load_runtime_config_json(config_path, runtime_config, config_error)) {
        std::printf("[Main] Loaded runtime config: %s\n", config_path);
    } else if (explicit_config) {
        std::fprintf(stderr, "[Main] Failed to load runtime config '%s': %s\n",
                     config_path, config_error.c_str());
        return 1;
    } else {
        std::fprintf(stderr, "[Main] WARNING: using built-in defaults; config '%s' not loaded: %s\n",
                     config_path, config_error.c_str());
    }

    if (check_config_only) {
        std::printf("[Main] Config OK\n");
        std::printf("[Main] vision.confidence_threshold=%.2f\n",
                    runtime_config.vision_confidence_threshold);
        std::printf("[Main] decision.zones=%zu  trajectories=%zu  reset=%.1fs  stable=%.1fs\n",
                    runtime_config.decision.zones.size(),
                    runtime_config.decision.trajectories.size(),
                    runtime_config.decision.reset_cooldown_s,
                    runtime_config.decision.stable_dart_time_s);
        return 0;
    }

    // ── Instantiate modules ───────────────────────────────────────────────
    CameraModule   camera(CAMERA_INDEX);
    VisionModule   vision(camera.buffer(), runtime_config.vision_confidence_threshold);
    ArmModule      arm;
    DecisionModule decision(vision.buffer(), arm, runtime_config.decision);

    // ── Start modules ────────────────────────────────────────────────────
    if (!camera.start()) {
        std::fprintf(stderr, "[Main] Camera failed to start\n");
        return 1;
    }

    vision.start();

    if (!arm.start()) {
        std::fprintf(stderr, "[Main] Arm failed to start — running in camera-only mode\n");
        // Continue without the arm so vision can still be tested
    }

    decision.start();

    std::printf("[Main] Running — press Ctrl-C to stop\n");

    // ── Spin until SIGINT ─────────────────────────────────────────────────
    while (!g_shutdown) {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(100ms);
    }

    // ── Shutdown in reverse order ─────────────────────────────────────────
    std::printf("\n[Main] Shutting down...\n");
    decision.stop();
    arm.stop();
    vision.stop();
    camera.stop();

    std::printf("[Main] Done\n");
    return 0;
}
