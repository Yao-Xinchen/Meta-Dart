// test/calibrate_gripper.cpp
//
// Manually calibrate gripper open/closed positions.
// Torque is disabled so you can move the gripper by hand.
//
// Usage:  sudo ./calibrate_gripper
//
//   1. Move gripper to fully OPEN by hand, press Enter
//   2. Move gripper to fully CLOSED by hand, press Enter
//   Program prints the constexpr values to put in config.hpp.

#include "config.hpp"

#include <DynamixelWorkbench.h>
#include <cstdio>

int main()
{
    const char* log = nullptr;
    DynamixelWorkbench wb;

    if (!wb.init(DXL_PORT, DXL_BAUD, &log)) {
        std::fprintf(stderr, "init failed: %s\n", log ? log : "?");
        return 1;
    }

    uint16_t model = 0;
    if (!wb.ping(DXL_ID_GRIPPER, &model, &log)) {
        std::fprintf(stderr, "ping failed for gripper ID %u: %s\n",
                     DXL_ID_GRIPPER, log ? log : "?");
        return 1;
    }
    std::printf("Gripper found: ID=%u  model=%u\n\n", DXL_ID_GRIPPER, model);

    // Use current-based position mode so the motor holds but won't stall
    // against mechanical limits — same mode used in production.
    wb.currentBasedPositionMode(DXL_ID_GRIPPER, 0, &log);
    std::printf("Torque ON (current-based) — push gripper to its limits by hand.\n\n");

    auto read_pos = [&]() -> float {
        float rad = 0.f;
        wb.getRadian(DXL_ID_GRIPPER, &rad);
        return rad;
    };

    // Open
    std::printf("Push gripper to FULLY OPEN against the mechanical stop, then press Enter...");
    std::fflush(stdout);
    std::getchar();
    float open_pos = read_pos();
    std::printf("  → OPEN  = %.4f rad\n\n", open_pos);

    // Closed
    std::printf("Push gripper to FULLY CLOSED against the mechanical stop, then press Enter...");
    std::fflush(stdout);
    std::getchar();
    float closed_pos = read_pos();
    std::printf("  → CLOSED = %.4f rad\n\n", closed_pos);

    std::printf("=== Update config.hpp with: ===\n");
    std::printf("  constexpr float GRIPPER_OPEN   = %.4ff;\n", open_pos);
    std::printf("  constexpr float GRIPPER_CLOSED = %.4ff;\n", closed_pos);

    return 0;
}
