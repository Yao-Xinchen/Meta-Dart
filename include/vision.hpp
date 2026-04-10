#pragma once

#include "triple_buffer.hpp"
#include "types.hpp"

#include <atomic>
#include <thread>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>

// ─────────────────────────────────────────────────────────────────────────────
// VisionModule
//
// Reads frames from the camera buffer, runs YOLO OBB dart detection via ONNX
// Runtime, and publishes Detection results.  Falls back to stub mode (always
// publishes Detection{valid=false}) if the ONNX model file is not found.
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

    // Resize BGR→RGB, normalize to [0,1], pack into NCHW float32 blob.
    std::vector<float> preprocess(const cv::Mat& bgr, int w, int h);

    // Parse output0 [1, 6, 8400]: return highest-conf detection above threshold.
    // Attribute layout per anchor d: data[attr * n_anchors + d]
    //   attr 0: cx, 1: cy, 2: w, 3: h, 4: angle_rad, 5: conf  (all in 640-space)
    struct OBBResult {
        bool  valid = false;
        float cx = 0.f, cy = 0.f;  // centre in YOLO input (640×640) pixel space
        float w  = 0.f, h  = 0.f;
        float angle = 0.f;          // radians
        float conf  = 0.f;
    };
    OBBResult postprocess(const float* data, int64_t n_anchors);

    // Pinhole projection: OBB centre in 640-space → world coords [m].
    void pixel_to_world(float cx640, float cy640,
                        int frame_w, int frame_h,
                        float& wx, float& wy) const;

    TripleBuffer<Frame>&    camera_buf_;
    TripleBuffer<Detection> buf_;
    std::thread             thread_;
    std::atomic<bool>       running_{false};

    Ort::Env        ort_env_;
    Ort::Session    ort_session_{nullptr};
    Ort::MemoryInfo ort_mem_;
    bool            model_loaded_ = false;
};
