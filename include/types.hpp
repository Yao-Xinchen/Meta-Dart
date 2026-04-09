#pragma once

#include <array>
#include <cstdint>
#include <opencv2/opencv.hpp>

// ─────────────────────────────────────────────────────────────────────────────
// Camera → Vision
// ─────────────────────────────────────────────────────────────────────────────
struct Frame {
    cv::Mat  image;
    int64_t  timestamp_us = 0;   // microseconds since epoch
};

// ─────────────────────────────────────────────────────────────────────────────
// Vision → Decision
// ─────────────────────────────────────────────────────────────────────────────
struct Detection {
    bool  valid = false;
    float x = 0.f, y = 0.f, z = 0.f;   // dart position in world frame [m]
    // z is the height above the arm base plane; for a fixed overhead camera
    // this is either known or assumed constant (dart resting on a flat surface).
};

// ─────────────────────────────────────────────────────────────────────────────
// Decision → Arm
// ─────────────────────────────────────────────────────────────────────────────
struct ArmCmd {
    enum class Type {
        Idle,
        MoveJoint,   // command joints[0..3] directly (radians)
        MovePose,    // IK from (px,py,pz) + desired end-effector pitch
        Grip,        // close gripper
        Release,     // open gripper
    } type = Type::Idle;

    std::array<float, 4> joints  = {};   // [J1,J2,J3,J4] radians for MoveJoint
    float px = 0.f, py = 0.f, pz = 0.f; // Cartesian target [m] for MovePose
    float pitch = 0.f;                   // desired EE pitch [rad] for MovePose
    float current_limit = 0.f;          // 0 → use motor default
};

// ─────────────────────────────────────────────────────────────────────────────
// Arm → Decision
// ─────────────────────────────────────────────────────────────────────────────
struct ArmState {
    std::array<float, 4> joints = {};   // present joint angles [rad]
    float px = 0.f, py = 0.f, pz = 0.f; // FK end-effector position [m]
    bool  reached_goal = false;
    bool  hw_ok        = false;         // false if Dynamixel init failed
};
