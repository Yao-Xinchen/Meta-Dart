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
        ServoJoint,  // send joints[0..3] directly; caller owns trajectory timing
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
        Stretch,      // normal pre-arm: stretch, latch externally, then retract
        Retract,      // return both motors to their zero/home positions
        TakeLoad,     // move back to stretched position and hold spring load
        SafeRetract,  // slowly return home after trigger release
    } type = Type::Idle;
};

enum class SpringStretcherPhase {
    Offline,
    Calibrating,
    Home,
    Stretching,
    HoldingStretched,
    Retracting,
    ArmedAndClear,
    TakingLoad,
    SafeRetracting,
    Fault,
};

// ─────────────────────────────────────────────────────────────────────────────
// Spring stretcher motors → Decision
// ─────────────────────────────────────────────────────────────────────────────
struct SpringStretcherState {
    std::array<float, 2> position = {};  // left/right cumulative motor angle [rad]
    std::array<float, 2> velocity = {};  // left/right motor velocity [rad/s]
    std::array<float, 2> current  = {};  // left/right command/feedback current [A]
    SpringStretcherPhase phase = SpringStretcherPhase::Offline;
    bool reached_goal = false;
    bool armed_and_clear = false;
    bool holding_load = false;
    bool calibrated   = false;
    bool hw_ok        = false;
};

// ─────────────────────────────────────────────────────────────────────────────
// Decision → trigger motor
// ─────────────────────────────────────────────────────────────────────────────
struct TriggerCmd {
    enum class Type {
        Hold,     // bar high: latch/hold spring
        Release,  // bar low: release spring
    } type = Type::Hold;
};

// ─────────────────────────────────────────────────────────────────────────────
// Trigger motor → Decision
// ─────────────────────────────────────────────────────────────────────────────
struct TriggerState {
    enum class Position {
        Unknown,
        HoldingHigh,
        ReleasingLow,
    } position = Position::Unknown;

    bool settled = false;      // inferred from command age; no sensor in v1
    bool hw_ok = false;        // TODO hardware interface currently reports true
    bool inferred = true;      // state is command/time inferred, not sensed
};
