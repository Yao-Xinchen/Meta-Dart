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

void CameraModule::loop() {
    cv::VideoCapture cap(device_index_, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::fprintf(stderr, "[Camera] Failed to open device %d\n", device_index_);
        running_ = false;
        return;
    }

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
