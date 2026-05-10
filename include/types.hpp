#pragma once

#include <array>

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
        MoveJoint,   // linearly interpolate joints[0..3] over duration seconds
        Grip,        // close gripper
        Release,     // open gripper
    } type = Type::Idle;

    std::array<float, 4> joints   = {};  // [J1,J2,J3,J4] target angles [rad]
    float duration    = 0.f;             // motion duration [s]; 0 → 2.0 s default
    float current_limit = 0.f;           // 0 → use motor default
};

// ─────────────────────────────────────────────────────────────────────────────
// Arm → Decision
// ─────────────────────────────────────────────────────────────────────────────
struct ArmState {
    std::array<float, 4> joints = {};   // present joint angles [rad]
    bool  reached_goal = false;
    bool  hw_ok        = false;         // false if Dynamixel init failed
};

// ─────────────────────────────────────────────────────────────────────────────
// Decision → spring stretcher motors
// ─────────────────────────────────────────────────────────────────────────────
struct SpringStretcherCmd {
    enum class Type {
        Idle,
        Stretch,    // pull both springs to the configured stretched positions
        Retract,    // return both motors to their zero/home positions
    } type = Type::Idle;
};

// ─────────────────────────────────────────────────────────────────────────────
// Spring stretcher motors → Decision
// ─────────────────────────────────────────────────────────────────────────────
struct SpringStretcherState {
    std::array<float, 2> position = {};  // left/right cumulative motor angle [rad]
    std::array<float, 2> velocity = {};  // left/right motor velocity [rad/s]
    std::array<float, 2> current  = {};  // left/right command/feedback current [A]
    bool reached_goal = false;
    bool hw_ok        = false;
};
