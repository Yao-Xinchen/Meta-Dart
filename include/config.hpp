#pragma once

#include <cstdint>

// ── Serial / Dynamixel ────────────────────────────────────────────────────────
constexpr const char* DXL_PORT     = "/dev/ttyUSB0";
constexpr int         DXL_BAUD     = 57600;

// Motor IDs (OpenManipulator-X defaults)
constexpr uint8_t DXL_ID_J1       = 11;   // base rotation
constexpr uint8_t DXL_ID_J2       = 12;   // shoulder
constexpr uint8_t DXL_ID_J3       = 13;   // elbow
constexpr uint8_t DXL_ID_J4       = 14;   // wrist pitch
constexpr uint8_t DXL_ID_GRIPPER  = 15;

// ── Camera ────────────────────────────────────────────────────────────────────
constexpr int CAMERA_INDEX         = 2;    // matches temp/camera.cpp

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
constexpr float GRIPPER_OPEN      = -0.010f;
constexpr float GRIPPER_CLOSED    =  0.010f;

// ── Control loop timing ───────────────────────────────────────────────────────
constexpr int ARM_LOOP_HZ          = 50;   // arm control thread rate
constexpr int VISION_LOOP_HZ       = 30;   // vision thread rate
constexpr int DECISION_LOOP_HZ     = 10;   // decision thread rate

// Goal-reached threshold [rad]
constexpr float GOAL_REACHED_THRESH = 0.02f;

// ── Vision / ONNX ─────────────────────────────────────────────────────────────
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
