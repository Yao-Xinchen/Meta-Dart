#pragma once

#include <array>
#include <optional>

// ─────────────────────────────────────────────────────────────────────────────
// Kinematics for OpenManipulator-X (4-DOF)
//
// Joint convention:
//   J1 – base yaw  (rotates the whole arm around Z)
//   J2 – shoulder pitch (in the vertical plane)
//   J3 – elbow pitch
//   J4 – wrist pitch
//
// Link lengths (metres) from config.hpp:
//   L1 = 0.077  (base → shoulder, vertical)
//   L2 = 0.130  (upper arm)
//   L3 = 0.124  (forearm)
//   L4 = 0.126  (wrist to tip)
//
// Frame convention: X forward, Y left, Z up.  All angles in radians.
// ─────────────────────────────────────────────────────────────────────────────

struct Pose {
    float x, y, z;    // end-effector position [m]
    float pitch;       // end-effector pitch angle [rad] (0 = horizontal, -π/2 = pointing down)
};

using Joints = std::array<float, 4>;

// Forward kinematics: joint angles → end-effector Pose
Pose fk(const Joints& q);

// Inverse kinematics: desired Pose → joint angles
// Returns std::nullopt if the pose is unreachable or outside joint limits.
// When multiple elbow solutions exist (elbow-up / elbow-down), elbow-up is preferred.
std::optional<Joints> ik(const Pose& target);
