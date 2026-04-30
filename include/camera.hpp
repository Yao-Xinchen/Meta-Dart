#pragma once

#include "triple_buffer.hpp"

#include <atomic>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <thread>

struct Frame {
    cv::Mat  image;
    int64_t  timestamp_us = 0;   // microseconds since epoch
};

// ─────────────────────────────────────────────────────────────────────────────
// CameraModule
//
// Captures frames from an OpenCV-compatible camera device and pushes them
// into a TripleBuffer<Frame> for downstream consumers (VisionModule).
// ─────────────────────────────────────────────────────────────────────────────
class CameraModule {
public:
    explicit CameraModule(int device_index);
    ~CameraModule();

    // Open the device and start the capture thread.
    // Returns false if the camera cannot be opened.
    bool start();

    // Signal the thread to stop and wait for it to exit.
    void stop();

    TripleBuffer<Frame>& buffer() { return buf_; }

private:
    void loop();

    int                   device_index_;
    TripleBuffer<Frame>   buf_;
    std::thread           thread_;
    std::atomic<bool>     running_{false};
};
