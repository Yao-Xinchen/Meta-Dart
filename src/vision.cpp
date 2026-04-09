#include "vision.hpp"
#include "config.hpp"

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

VisionModule::VisionModule(TripleBuffer<Frame>& camera_buf)
    : camera_buf_(camera_buf) {}

VisionModule::~VisionModule() { stop(); }

bool VisionModule::start() {
    running_ = true;
    thread_  = std::thread(&VisionModule::loop, this);
    return true;
}

void VisionModule::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void VisionModule::loop() {
    constexpr auto period = std::chrono::microseconds(1'000'000 / VISION_LOOP_HZ);
    using Clock = std::chrono::steady_clock;

    Frame     frame;
    Detection det;

    while (running_) {
        auto t0 = Clock::now();

        if (camera_buf_.read(frame)) {
            // ── TODO: implement dart detection ────────────────────────────
            //
            // Step 1 – Detect dart in image (OpenCV example outline):
            //   cv::Mat hsv;
            //   cv::cvtColor(frame.image, hsv, cv::COLOR_BGR2HSV);
            //   cv::Mat mask;
            //   cv::inRange(hsv, lower_white, upper_white, mask);
            //   std::vector<std::vector<cv::Point>> contours;
            //   cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            //   // pick largest contour, find centroid → pixel (u, v)
            //
            // Step 2 – Project pixel → world (pinhole model, fixed camera height H):
            //   float fx = ..., fy = ..., cx = ..., cy = ...;  // from calibration
            //   det.x = (u - cx) / fx * H;
            //   det.y = (v - cy) / fy * H;
            //   det.z = DART_REST_HEIGHT;  // constant for flat surface
            //   det.valid = true;
            //
            // ─────────────────────────────────────────────────────────────

            det.valid = false;  // stub: no detection yet
        }

        buf_.write(det);

        std::this_thread::sleep_until(t0 + period);
    }
}
