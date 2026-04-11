#include "vision.hpp"
#include "config.hpp"

#include <chrono>
#include <thread>
#include <cstdio>
#include <array>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────────────────────
// Construction / destruction
// ─────────────────────────────────────────────────────────────────────────────

VisionModule::VisionModule(TripleBuffer<Frame>& camera_buf)
    : camera_buf_(camera_buf)
    , ort_env_(ORT_LOGGING_LEVEL_WARNING, "vision")
    , ort_mem_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
{
    // Gracefully fall back to stub mode if the model file is missing.
    std::FILE* f = std::fopen(ONNX_MODEL_PATH, "rb");
    if (!f) {
        std::fprintf(stderr,
            "[Vision] WARNING: ONNX model not found at '%s'. "
            "Running in stub mode (no detections).\n", ONNX_MODEL_PATH);
        return;
    }
    std::fclose(f);

    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(0);  // 0 = let ORT choose based on hardware
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    try {
        ort_session_ = Ort::Session(ort_env_, ONNX_MODEL_PATH, opts);
        model_loaded_ = true;
        std::printf("[Vision] ONNX model loaded from '%s'\n", ONNX_MODEL_PATH);
    } catch (const Ort::Exception& e) {
        std::fprintf(stderr,
            "[Vision] Failed to load ONNX model: %s. Running in stub mode.\n",
            e.what());
    }
}

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

// ─────────────────────────────────────────────────────────────────────────────
// Preprocessing: BGR frame → NCHW float32 blob in [0, 1]
// ─────────────────────────────────────────────────────────────────────────────

std::vector<float> VisionModule::preprocess(const cv::Mat& bgr, int w, int h)
{
    cv::Mat resized, rgb, f32;
    cv::resize(bgr, resized, cv::Size(w, h), 0, 0, cv::INTER_LINEAR);
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(f32, CV_32FC3, 1.0 / 255.0);

    // HWC → NCHW (batch=1)
    std::vector<float> blob(1 * 3 * h * w);
    const int plane = h * w;
    for (int row = 0; row < h; ++row) {
        const float* src = f32.ptr<float>(row);
        for (int col = 0; col < w; ++col) {
            int idx = row * w + col;
            blob[0 * plane + idx] = src[col * 3 + 0];  // R
            blob[1 * plane + idx] = src[col * 3 + 1];  // G
            blob[2 * plane + idx] = src[col * 3 + 2];  // B
        }
    }
    return blob;
}

// ─────────────────────────────────────────────────────────────────────────────
// Postprocessing: pick highest-confidence dart detection
// ─────────────────────────────────────────────────────────────────────────────

VisionModule::OBBResult VisionModule::postprocess(const float* data,
                                                   int64_t n_anchors)
{
    // Ultralytics OBB output layout [6, n_anchors]:
    //   row 0: cx, 1: cy, 2: w, 3: h, 4: class_conf, 5: angle_rad
    OBBResult best;
    for (int64_t d = 0; d < n_anchors; ++d) {
        float conf = data[4 * n_anchors + d];
        if (conf < CONF_THRESHOLD || conf <= best.conf) continue;
        best.valid = true;
        best.cx    = data[0 * n_anchors + d];
        best.cy    = data[1 * n_anchors + d];
        best.w     = data[2 * n_anchors + d];
        best.h     = data[3 * n_anchors + d];
        best.conf  = conf;
        best.angle = data[5 * n_anchors + d];
    }
    return best;
}

// ─────────────────────────────────────────────────────────────────────────────
// Pixel → world (pinhole model, camera pointing straight down)
// ─────────────────────────────────────────────────────────────────────────────

void VisionModule::pixel_to_world(float cx640, float cy640,
                                   int frame_w, int frame_h,
                                   float& wx, float& wy) const
{
    // Scale OBB centre from 640-space back to native frame resolution
    float px = cx640 * (static_cast<float>(frame_w) / YOLO_INPUT_W);
    float py = cy640 * (static_cast<float>(frame_h) / YOLO_INPUT_H);

    // Pinhole: X = (u - cx) / fx * H,  Y = (v - cy) / fy * H
    wx = (px - CAM_CX) / CAM_FX * CAM_HEIGHT_M;
    wy = (py - CAM_CY) / CAM_FY * CAM_HEIGHT_M;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main loop
// ─────────────────────────────────────────────────────────────────────────────

void VisionModule::loop()
{
    constexpr auto period = std::chrono::microseconds(1'000'000 / VISION_LOOP_HZ);
    using Clock = std::chrono::steady_clock;

    const std::array<int64_t, 4> input_shape{1, 3, YOLO_INPUT_H, YOLO_INPUT_W};
    const char* input_name  = "images";
    const char* output_name = "output0";

    Frame     frame;
    Detection det;

    // Timing stats (printed every N_STAT frames, controlled by VISION_TIMING_DEBUG)
    constexpr int N_STAT = 30;
    int    stat_n      = 0;
    double sum_pre     = 0, sum_infer = 0, sum_vis = 0, sum_total = 0;

    auto ms = [](auto dur) {
        return std::chrono::duration<double, std::milli>(dur).count();
    };

    while (running_) {
        auto t0 = Clock::now();
        det.valid = false;

        double dt_pre = 0, dt_infer = 0, dt_vis = 0;

        if (camera_buf_.read(frame) && !frame.image.empty()) {
            if (model_loaded_) {
                // ── Preprocess ───────────────────────────────────────────────
                auto tp0  = Clock::now();
                auto blob = preprocess(frame.image, YOLO_INPUT_W, YOLO_INPUT_H);
                dt_pre    = ms(Clock::now() - tp0);

                // ── Inference ────────────────────────────────────────────────
                auto ti0 = Clock::now();
                Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
                    ort_mem_, blob.data(), blob.size(),
                    input_shape.data(), input_shape.size());

                auto outputs = ort_session_.Run(
                    Ort::RunOptions{nullptr},
                    &input_name,  &input_tensor,  1,
                    &output_name, 1);
                dt_infer = ms(Clock::now() - ti0);

                const float* raw      = outputs[0].GetTensorData<float>();
                int64_t      n_anchor = outputs[0].GetTensorTypeAndShapeInfo().GetShape()[2];

                // ── Detection ────────────────────────────────────────────────
                OBBResult obb = postprocess(raw, n_anchor);

                // ── Visualization ─────────────────────────────────────────────
                auto tv0 = Clock::now();
                cv::Mat vis = frame.image.clone();

                if (obb.valid) {
                    pixel_to_world(obb.cx, obb.cy,
                                   frame.image.cols, frame.image.rows,
                                   det.x, det.y);
                    det.z     = DART_REST_HEIGHT_M;
                    det.valid = true;

                    float sx = static_cast<float>(frame.image.cols) / YOLO_INPUT_W;
                    float sy = static_cast<float>(frame.image.rows) / YOLO_INPUT_H;

                    cv::RotatedRect rrect(
                        cv::Point2f(obb.cx * sx, obb.cy * sy),
                        cv::Size2f(obb.w * sx, obb.h * sy),
                        obb.angle * (180.0f / 3.14159265f));

                    cv::Point2f corners[4];
                    rrect.points(corners);
                    for (int i = 0; i < 4; ++i)
                        cv::line(vis, corners[i], corners[(i + 1) % 4],
                                 cv::Scalar(0, 255, 0), 2);
                    cv::circle(vis, rrect.center, 5, cv::Scalar(0, 0, 255), -1);

                    char label[80];
                    std::snprintf(label, sizeof(label),
                                  "conf=%.2f  (%.3fm, %.3fm)",
                                  obb.conf, det.x, det.y);
                    cv::putText(vis, label, cv::Point(10, 30),
                                cv::FONT_HERSHEY_SIMPLEX, 0.7,
                                cv::Scalar(255, 255, 0), 2);
                }

                cv::imshow("Vision", vis);
                cv::waitKey(1);
                dt_vis = ms(Clock::now() - tv0);
            }
        }

        buf_.write(det);

        double dt_total = ms(Clock::now() - t0);
        sum_pre   += dt_pre;
        sum_infer += dt_infer;
        sum_vis   += dt_vis;
        sum_total += dt_total;

        if (VISION_TIMING_DEBUG && ++stat_n == N_STAT) {
            double n = N_STAT;
            std::printf("[Vision timing over %d frames]  "
                        "pre=%.1fms  infer=%.1fms  vis=%.1fms  "
                        "total=%.1fms  budget=%.1fms\n",
                        N_STAT,
                        sum_pre / n, sum_infer / n, sum_vis / n,
                        sum_total / n,
                        ms(period));
            stat_n = sum_pre = sum_infer = sum_vis = sum_total = 0;
        }

        std::this_thread::sleep_until(t0 + period);
    }

    cv::destroyAllWindows();
}
