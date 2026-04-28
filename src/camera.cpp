#include "camera.hpp"
#include "config.hpp"

#include <chrono>
#include <cstdio>
#include <opencv2/opencv.hpp>

static int64_t now_us() {
    using namespace std::chrono;
    return duration_cast<microseconds>(
        steady_clock::now().time_since_epoch()).count();
}

CameraModule::CameraModule(int device_index)
    : device_index_(device_index) {}

CameraModule::~CameraModule() {
    stop();
}

bool CameraModule::start() {
    running_ = true;
    thread_  = std::thread(&CameraModule::loop, this);
    return true;  // actual open failure is reported in loop() via stderr
}

void CameraModule::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

static void apply_camera_settings(cv::VideoCapture& cap) {
    // V4L2 exposure_auto: 1=manual, 3=aperture-priority (auto)
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, CAM_AUTO_EXPOSURE ? 3 : 1);
    if (!CAM_AUTO_EXPOSURE)
        cap.set(cv::CAP_PROP_EXPOSURE, CAM_EXPOSURE);

    auto try_set = [&](int prop, double val, const char* name) {
        if (val < 0) return;
        if (!cap.set(prop, val))
            std::fprintf(stderr, "[Camera] WARNING: could not set %s to %.0f\n", name, val);
    };

    try_set(cv::CAP_PROP_BRIGHTNESS,   CAM_BRIGHTNESS,  "brightness");
    try_set(cv::CAP_PROP_CONTRAST,     CAM_CONTRAST,    "contrast");
    try_set(cv::CAP_PROP_SATURATION,   CAM_SATURATION,  "saturation");
    try_set(cv::CAP_PROP_GAIN,         CAM_GAIN,        "gain");

    std::printf("[Camera] exposure=%s(%.0f) brightness=%.0f contrast=%.0f saturation=%.0f\n",
                CAM_AUTO_EXPOSURE ? "auto" : "manual ",
                CAM_AUTO_EXPOSURE ? 0.0 : CAM_EXPOSURE,
                CAM_BRIGHTNESS, CAM_CONTRAST, CAM_SATURATION);
}

void CameraModule::loop() {
    cv::VideoCapture cap(device_index_, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::fprintf(stderr, "[Camera] Failed to open device %d\n", device_index_);
        running_ = false;
        return;
    }

    apply_camera_settings(cap);

    Frame frame;
    while (running_) {
        if (!cap.read(frame.image)) {
            std::fprintf(stderr, "[Camera] Failed to read frame\n");
            break;
        }
        frame.timestamp_us = now_us();
        buf_.write(frame);
    }
}
