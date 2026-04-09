#include "config.hpp"
#include "camera.hpp"
#include "vision.hpp"
#include "arm.hpp"
#include "decision.hpp"

#include <csignal>
#include <cstdio>
#include <atomic>

// ─────────────────────────────────────────────────────────────────────────────
static std::atomic<bool> g_shutdown{false};

static void on_signal(int) {
    g_shutdown = true;
}

// ─────────────────────────────────────────────────────────────────────────────
int main() {
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    std::printf("[Main] Starting loader\n");

    // ── Instantiate modules ───────────────────────────────────────────────
    CameraModule   camera(CAMERA_INDEX);
    VisionModule   vision(camera.buffer());
    ArmModule      arm;
    DecisionModule decision(vision.buffer(), arm.state_buf(), arm.cmd_buf());

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
