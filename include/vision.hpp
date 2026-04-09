#pragma once

#include "triple_buffer.hpp"
#include "types.hpp"

#include <atomic>
#include <thread>

// ─────────────────────────────────────────────────────────────────────────────
// VisionModule
//
// Reads frames from the camera buffer, runs dart detection, and publishes
// Detection results.
//
// Current state: STUB — always publishes Detection{valid=false}.
//
// TODO (vision implementation):
//   1. Dart detection: segment the white dart from the background using
//      HSV thresholding or contour detection on the overhead view.
//   2. Pixel → world: given known camera height and intrinsics, project the
//      detected pixel centroid to a 3-D world coordinate via a homography
//      or simple pinhole model (z is fixed = dart resting height).
//   3. For VLA: replace the OpenCV block with an inference call and keep
//      everything else (buffers, threading) unchanged.
// ─────────────────────────────────────────────────────────────────────────────
class VisionModule {
public:
    explicit VisionModule(TripleBuffer<Frame>& camera_buf);
    ~VisionModule();

    bool start();
    void stop();

    TripleBuffer<Detection>& buffer() { return buf_; }

private:
    void loop();

    TripleBuffer<Frame>&    camera_buf_;
    TripleBuffer<Detection> buf_;
    std::thread             thread_;
    std::atomic<bool>       running_{false};
};
