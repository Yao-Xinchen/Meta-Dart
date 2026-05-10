// test/test_vision_camera.cpp
//
// Camera + YOLO/ONNX vision smoke test.
//
// Starts CameraModule and VisionModule only. It does not start ArmModule or
// DecisionModule, so it will not move the robot.
//
// Usage:
//   ./test_vision_camera [duration_seconds]
//
// With no duration, runs until Ctrl-C. VisionModule opens an OpenCV window named
// "Vision" and overlays the best OBB detection from models/best.onnx.

#include "camera.hpp"
#include "config.hpp"
#include "types.hpp"
#include "vision.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>

using namespace std::chrono_literals;

static std::atomic<bool> g_shutdown{false};

static void on_signal(int)
{
    g_shutdown = true;
}

static int parse_duration_seconds(int argc, char* argv[])
{
    if (argc < 2) return 0;
    int seconds = std::atoi(argv[1]);
    if (seconds < 0) seconds = 0;
    return seconds;
}

int main(int argc, char* argv[])
{
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    const int duration_s = parse_duration_seconds(argc, argv);

    std::printf("=== Vision camera test ===\n");
    std::printf("Camera index: %d (/dev/video%d)\n", CAMERA_INDEX, CAMERA_INDEX);
    std::printf("ONNX model: %s\n", ONNX_MODEL_PATH);
    if (duration_s > 0)
        std::printf("Duration: %d second(s)\n", duration_s);
    else
        std::printf("Duration: until Ctrl-C\n");
    std::printf("This test does not start the arm or decision module.\n\n");

    CameraModule camera(CAMERA_INDEX);
    VisionModule vision(camera.buffer());

    if (!camera.start()) {
        std::fprintf(stderr, "ERROR: camera.start() failed\n");
        return 1;
    }
    if (!vision.start()) {
        std::fprintf(stderr, "ERROR: vision.start() failed\n");
        camera.stop();
        return 1;
    }

    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    auto next_print = t0;

    Detection latest{};
    int updates = 0;
    int window_updates = 0;
    int window_valid = 0;
    Detection last_valid{};

    while (!g_shutdown) {
        const auto now = Clock::now();
        if (duration_s > 0 && now - t0 >= std::chrono::seconds(duration_s))
            break;

        if (vision.buffer().read(latest)) {
            ++updates;
            ++window_updates;
            if (latest.valid) {
                ++window_valid;
                last_valid = latest;
            }
        }

        if (now >= next_print) {
            if (window_valid > 0) {
                std::printf("[VisionTest] valid frames %d/%d: last x=%.3f m  y=%.3f m  z=%.3f m  total_updates=%d\n",
                            window_valid, window_updates,
                            last_valid.x, last_valid.y, last_valid.z, updates);
            } else {
                std::printf("[VisionTest] no valid frames in last window  window_updates=%d  total_updates=%d\n",
                            window_updates, updates);
            }
            window_updates = 0;
            window_valid = 0;
            next_print = now + 1s;
        }

        std::this_thread::sleep_for(20ms);
    }

    std::printf("\n[VisionTest] Shutting down...\n");
    vision.stop();
    camera.stop();
    std::printf("[VisionTest] Done\n");
    return 0;
}
