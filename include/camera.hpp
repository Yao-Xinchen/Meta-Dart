#pragma once

#include "triple_buffer.hpp"
#include "types.hpp"

#include <atomic>
#include <thread>

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
