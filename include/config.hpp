#pragma once

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

// ── Control loop timing ───────────────────────────────────────────────────────
constexpr int ARM_LOOP_HZ          = 100;  // arm control thread rate
constexpr int VISION_LOOP_HZ       = 30;   // vision thread rate
constexpr int DECISION_LOOP_HZ     = 10;   // decision thread rate
constexpr int SPRING_STRETCHER_LOOP_HZ = 500;  // DJI M3508 control thread rate

// Goal-reached threshold [rad]
constexpr float GOAL_REACHED_THRESH = 0.05f;
constexpr float GRIPPER_SETTLE_TIME_S = 0.5f;

// ── Arm smoothness / anti-jitter tuning ──────────────────────────────────────
// X-series Dynamixel raw profile units.  Lower values are gentler but slower.
// If the arm feels too sluggish, increase velocity first, then acceleration.
constexpr int32_t JOINT_PROFILE_VELOCITY     = 80;
constexpr int32_t JOINT_PROFILE_ACCELERATION = 20;

// Position controller gains.  Lower P reduces hunting/shaking at the cost of
// stiffness.  Defaults on many XM430 setups are much higher.
constexpr int32_t JOINT_POSITION_P_GAIN      = 350;
constexpr int32_t JOINT_POSITION_I_GAIN      = 0;
constexpr int32_t JOINT_POSITION_D_GAIN      = 0;

// Do not spam tiny Goal_Position updates smaller than this amount.  This
// removes high-frequency setpoint chatter near the beginning/end of S-curves.
constexpr float JOINT_GOAL_WRITE_DEADBAND    = 0.002f;  // rad

// ── Arm positions ─────────────────────────────────────────────────────────────
constexpr const char* POSITIONS_XML_PATH = "data/positions.xml";

// ── Dart launcher spring stretcher motors (DJI M3508 over SocketCAN) ─────────
constexpr const char* SPRING_STRETCHER_CAN_IFACE = "can0";
constexpr uint8_t SPRING_STRETCHER_LEFT_ID       = 1;     // DJI hardware ID, range 1-8
constexpr uint8_t SPRING_STRETCHER_RIGHT_ID      = 2;     // DJI hardware ID, range 1-8
constexpr float SPRING_STRETCHER_LEFT_DIR        = 1.0f;  // flip to -1 if wiring is mirrored
constexpr float SPRING_STRETCHER_RIGHT_DIR       = -1.0f; // opposite side usually mirrors
constexpr float SPRING_STRETCHER_REDUCTION_RATIO = 20.0f; // motor rotor angle / output shaft angle

// Tune these on hardware. Distances and velocities below are output shaft units.
constexpr float SPRING_STRETCHER_STRETCH_RAD     = 35.0f;
constexpr float SPRING_STRETCHER_HOME_RAD        = 0.0f;
constexpr float SPRING_STRETCHER_POS_TOL_RAD     = 0.08f;
constexpr float SPRING_STRETCHER_VEL_TOL_RAD_S   = 1.0f;
constexpr float SPRING_STRETCHER_HOLD_TIME_S     = 0.25f;

// Startup calibration: move each spring hook toward the top hard stop with a
// velocity loop and use the jammed position as home/zero. Positive stretch is
// motor.direction, so calibration moves in the opposite direction.
constexpr float SPRING_STRETCHER_CALIB_VEL_RAD_S = 2.0f;
constexpr float SPRING_STRETCHER_CALIB_MAX_CURRENT_A = 2.0f;
constexpr float SPRING_STRETCHER_CALIB_VEL_KP = 0.6f;
constexpr float SPRING_STRETCHER_CALIB_VEL_KD = 0.0000112f;
constexpr float SPRING_STRETCHER_CALIB_JAM_VEL_RAD_S = 1.0f;
constexpr float SPRING_STRETCHER_CALIB_JAM_TIME_S = 0.25f;
constexpr float SPRING_STRETCHER_CALIB_TIMEOUT_S = 10.0f;

// Cascaded position-to-velocity-to-current gains. Converted from
// Meta-Embedded dart STORE_ENERGY M3508 params:
// a2v {20, 0, 0.2, 100, 3000}, v2i {50, 0.1, 0.02, 2000, 6000}.
// Meta-Embedded uses output-shaft deg/deg-s at 8 ms and raw DJI current;
// this module uses output-shaft rad/rad-s at 2 ms and amps.
constexpr float SPRING_STRETCHER_POS_KP          = 20.0f;
constexpr float SPRING_STRETCHER_POS_KI          = 0.0f;
constexpr float SPRING_STRETCHER_POS_KD          = 0.0016f;
constexpr float SPRING_STRETCHER_VEL_KP          = 3.4971f;
constexpr float SPRING_STRETCHER_VEL_KI          = 0.8743f;
constexpr float SPRING_STRETCHER_VEL_KD          = 0.0000112f;
constexpr float SPRING_STRETCHER_MAX_VEL_RAD_S   = 3.0f;
constexpr float SPRING_STRETCHER_MAX_CURRENT_A   = 20.0f;

// ── Vision / ONNX ─────────────────────────────────────────────────────────────
constexpr bool        VISION_TIMING_DEBUG  = false;  // print per-frame timing stats
constexpr const char* ONNX_MODEL_PATH     = "models/best.onnx";
constexpr int         YOLO_INPUT_W        = 640;
constexpr int         YOLO_INPUT_H        = 640;
constexpr float       CONF_THRESHOLD      = 0.40f;
constexpr const char* VISION_WINDOW_NAME  = "Meta-Dart Vision";
constexpr int         VISION_WINDOW_W     = 1800;
constexpr int         VISION_WINDOW_H     = 1000;

// Camera intrinsics — replace with cv::calibrateCamera values (pixels, native res)
constexpr float CAM_FX             = 600.0f;
constexpr float CAM_FY             = 600.0f;
constexpr float CAM_CX             = 320.0f;   // ≈ width  / 2
constexpr float CAM_CY             = 240.0f;   // ≈ height / 2
constexpr float CAM_HEIGHT_M       = 0.50f;    // camera height above dart surface [m]
constexpr float DART_REST_HEIGHT_M = 0.00f;    // dart z in world frame [m]

// ── Debug: force-inject a detection at a fixed slot ──────────────────────────
// Flip FORCE_DETECTION to true to bypass ONNX and always report a dart at the
// configured world coordinates.  Dead-code-eliminated when false.
constexpr bool  FORCE_DETECTION      = false;
constexpr float FORCE_DETECTION_X    = 0.105f;  // slot_3 center x [m]
constexpr float FORCE_DETECTION_Y    = -0.008f; // slot_3 center y [m]

// ── Launch trigger MG995 servo over Linux sysfs PWM ──────────────────────────
// High keeps the bar/latch up. Low pulls the wire and releases the spring.
constexpr const char* TRIGGER_PWM_CHIP      = "auto";  // auto/pin18/pwm0-m1 or /sys/class/pwm/pwmchipN
constexpr int TRIGGER_PWM_CHANNEL           = 0;
constexpr float TRIGGER_HOLD_POS_DEG        = 100.0f;
constexpr float TRIGGER_RELEASE_POS_DEG     = 60.0f;
constexpr float TRIGGER_SETTLE_TIME_S       = 0.25f;
constexpr float TRIGGER_RELEASE_PULSE_S     = 0.35f;
constexpr float TRIGGER_SAFE_RELEASE_WAIT_S = 0.50f;
