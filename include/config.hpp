#pragma once

#include <array>
#include <cstdint>

// ── Serial / Dynamixel ────────────────────────────────────────────────────────
constexpr const char* DXL_PORT     = "/dev/ttyUSB0";
constexpr int         DXL_BAUD     = 1000000;

// Motor IDs (OpenManipulator-X defaults)
constexpr uint8_t DXL_ID_J1       = 11;   // base rotation
constexpr uint8_t DXL_ID_J2       = 12;   // shoulder
constexpr uint8_t DXL_ID_J3       = 13;   // elbow
constexpr uint8_t DXL_ID_J4       = 14;   // wrist pitch
constexpr uint8_t DXL_ID_GRIPPER  = 15;

// ── Camera ────────────────────────────────────────────────────────────────────
constexpr int CAMERA_INDEX         = 0;    // matches temp/camera.cpp

// Set any value to -1 to leave at driver default.
constexpr bool   CAM_AUTO_EXPOSURE   = true;   // false=manual(1), true=aperture-priority(3)
constexpr double CAM_EXPOSURE        = 50;    // exposure_time_absolute: 5–2500 (ignored when auto)
constexpr double CAM_BRIGHTNESS      = 50;     // 0–100, default=50
constexpr double CAM_CONTRAST        = 50;    // 0–100, default=50
constexpr double CAM_SATURATION      = 50;     // 0–100, default=50
constexpr double CAM_GAIN            = -1;     // 0–255, default=255; -1 = driver default

// ── Arm geometry (OpenManipulator-X DH link lengths, metres) ─────────────────
// L1: base-to-shoulder vertical offset
// L2: upper arm
// L3: forearm
// L4: wrist-to-tip
constexpr float L1 = 0.077f;
constexpr float L2 = 0.130f;
constexpr float L3 = 0.124f;
constexpr float L4 = 0.126f;

// Joint limits [rad] – from OpenManipulator-X datasheet
constexpr float J1_MIN = -2.827f,  J1_MAX =  2.827f;
constexpr float J2_MIN = -1.745f,  J2_MAX =  1.919f;
constexpr float J3_MIN = -0.873f,  J3_MAX =  1.571f;
constexpr float J4_MIN = -1.745f,  J4_MAX =  2.094f;

// Gripper open/close positions [rad]
// Gripper positions in metres — converted to raw encoder units at runtime
// using the same scale/offset as the OpenManipulator ros2_control plugin.
constexpr float GRIPPER_OPEN      =  0.019f;   // m
constexpr float GRIPPER_CLOSED    = -0.010f;   // m

// ros2_control unit-info parameters for motor ID 15
constexpr float GRIPPER_SCALE     =  0.000017626621790f;  // m / encoder_unit
constexpr float GRIPPER_OFFSET    = -0.036099321f;         // m
constexpr int32_t GRIPPER_CURRENT =  200;                  // Goal_Current raw value

// ── Pick slots: spatial regions + calibrated joint tables ────────────────────
// Add a slot by appending one entry. Set calibrated=true once hover_joints
// and pick_joints have been measured on hardware.
struct PickSlotConfig {
    bool  enabled;
    float x_min, x_max, y_min, y_max;
    bool  calibrated;
    std::array<float, 4> hover_joints;
    std::array<float, 4> pick_joints;
};

constexpr std::array<PickSlotConfig, 4> PICK_SLOTS = {{
    { true,  -0.202f, -0.142f, -0.019f,  0.041f,
      false, {0.f, -1.05f, 0.35f, 0.70f}, {0.f, -1.05f, 0.35f, 0.70f} }, // Position1
    { true,  -0.157f, -0.097f,  0.066f,  0.126f, 
      false, {}, {} }, // Position2
    { true,   0.107f,  0.167f,  0.068f,  0.128f,
      false, {}, {} }, // Position3
    { true,   0.146f,  0.206f, -0.016f,  0.044f,
      false, {}, {} }, // Position4
}};

constexpr int GRIPPER_CLOSE_SETTLE_MS = 700;
constexpr int GRIPPER_OPEN_SETTLE_MS  = 500;

// Vision-only region test mode.
// true  = skip all arm movement and only classify/print regions
// false = run the normal arm decision flow
constexpr bool ALLOW_REGION_REPORTING_WITHOUT_ARM_HW = true;

// ── Control loop timing ───────────────────────────────────────────────────────
constexpr int ARM_LOOP_HZ          = 50;   // arm control thread rate
constexpr int VISION_LOOP_HZ       = 30;   // vision thread rate
constexpr int DECISION_LOOP_HZ     = 10;   // decision thread rate

// Goal-reached threshold [rad]
constexpr float GOAL_REACHED_THRESH = 0.02f;

// ── Vision / ONNX ─────────────────────────────────────────────────────────────
constexpr bool        VISION_TIMING_DEBUG  = false;  // print per-frame timing stats
constexpr const char* ONNX_MODEL_PATH     = "models/best.onnx";
constexpr int         YOLO_INPUT_W        = 640;
constexpr int         YOLO_INPUT_H        = 640;
constexpr float       CONF_THRESHOLD      = 0.40f;

// Camera intrinsics — replace with cv::calibrateCamera values (pixels, native res)
constexpr float CAM_FX             = 600.0f;
constexpr float CAM_FY             = 600.0f;
constexpr float CAM_CX             = 320.0f;   // ≈ width  / 2
constexpr float CAM_CY             = 240.0f;   // ≈ height / 2
constexpr float CAM_HEIGHT_M       = 0.50f;    // camera height above dart surface [m]
constexpr float DART_REST_HEIGHT_M = 0.00f;    // dart z in world frame [m]
