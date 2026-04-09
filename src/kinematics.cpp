#include "kinematics.hpp"
#include "config.hpp"

#include <cmath>
#include <algorithm>

// ─────────────────────────────────────────────────────────────────────────────
// Forward kinematics
// ─────────────────────────────────────────────────────────────────────────────
Pose fk(const Joints& q) {
    // Joint 1 sets the azimuth plane.
    // Joints 2-4 are all pitch joints in that plane.
    // The shoulder is offset vertically by L1 from the base origin.

    float s1 = std::sin(q[0]), c1 = std::cos(q[0]);

    // Sum of pitch angles in the vertical plane
    float sum234 = q[1] + q[2] + q[3];

    // Reach in the horizontal plane and height
    float r = L2 * std::cos(q[1])
            + L3 * std::cos(q[1] + q[2])
            + L4 * std::cos(sum234);

    float z = L1
            + L2 * std::sin(q[1])
            + L3 * std::sin(q[1] + q[2])
            + L4 * std::sin(sum234);

    return Pose{
        .x     = r * c1,
        .y     = r * s1,
        .z     = z,
        .pitch = sum234,
    };
}

// ─────────────────────────────────────────────────────────────────────────────
// Inverse kinematics (analytical)
// ─────────────────────────────────────────────────────────────────────────────
std::optional<Joints> ik(const Pose& target) {
    // ── Joint 1 (base yaw) ────────────────────────────────────────────────
    float j1 = std::atan2(target.y, target.x);

    // ── Wrist position in the vertical plane ─────────────────────────────
    float r_ee = std::sqrt(target.x * target.x + target.y * target.y);
    float z_ee = target.z;

    // Back out the wrist centre (remove L4 contribution along pitch direction)
    float rw = r_ee - L4 * std::cos(target.pitch);
    float zw = z_ee - L1 - L4 * std::sin(target.pitch);

    // ── 2-R IK for joints 2 & 3 using the cosine rule ────────────────────
    float D = (rw * rw + zw * zw - L2 * L2 - L3 * L3) / (2.f * L2 * L3);

    if (D < -1.f || D > 1.f) return std::nullopt;  // unreachable

    // Elbow-up solution (negative acos gives elbow above the line)
    float j3_up   = std::acos(std::clamp(D, -1.f, 1.f));
    float j3_down = -j3_up;

    auto solve_j2 = [&](float j3) -> float {
        float k1 = L2 + L3 * std::cos(j3);
        float k2 = L3 * std::sin(j3);
        return std::atan2(zw, rw) - std::atan2(k2, k1);
    };

    // Try elbow-up first
    for (float j3 : {j3_up, j3_down}) {
        float j2 = solve_j2(j3);
        float j4 = target.pitch - j2 - j3;

        // Check joint limits
        if (j1 < J1_MIN || j1 > J1_MAX) continue;
        if (j2 < J2_MIN || j2 > J2_MAX) continue;
        if (j3 < J3_MIN || j3 > J3_MAX) continue;
        if (j4 < J4_MIN || j4 > J4_MAX) continue;

        return Joints{j1, j2, j3, j4};
    }

    return std::nullopt;  // both solutions violate joint limits
}
