// test/test_arm.cpp
//
// Staged hardware verification for ArmModule.
// Build with the test_arm CMake target, then run as root (or with dialout group):
//
//   sudo ./test_arm [stage]
//
// Stages (run in order):
//   A  – init & ping
//   B  – move to home (all joints = 0)
//   C  – gripper open / close
//   D  – single-axis sweeps (J1 … J4, ±SWEEP_RAD, then back to 0)
//   E  – FK / IK round-trip accuracy
//   F  – MovePose command
//
// If no stage argument is given, ALL stages run in sequence.

#include "arm.hpp"
#include "config.hpp"
#include "kinematics.hpp"
#include "types.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>

using namespace std::chrono_literals;

// ── tunables ──────────────────────────────────────────────────────────────────
static constexpr float SWEEP_RAD      = 0.3f;    // per-joint sweep amplitude [rad]
static constexpr float IK_TARGET_X    = 0.30f;   // reachable Cartesian target [m]
static constexpr float IK_TARGET_Y    = 0.00f;
static constexpr float IK_TARGET_Z    = 0.15f;
static constexpr float IK_TARGET_PITCH = 0.0f;
static constexpr auto  MOVE_TIMEOUT   = 5s;      // max wait for reached_goal

// ── helpers ───────────────────────────────────────────────────────────────────
static ArmState poll_state(ArmModule& arm)
{
    static ArmState last{};
    arm.state_buf().read(last);  // no-op if nothing new; last keeps prior value
    return last;
}

// Block until reached_goal or timeout. Returns true on success.
static bool wait_goal(ArmModule& arm, std::chrono::seconds timeout = MOVE_TIMEOUT)
{
    using Clock = std::chrono::steady_clock;
    auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
        ArmState s = poll_state(arm);
        if (s.reached_goal) return true;
        std::this_thread::sleep_for(50ms);
    }
    return false;
}

static void send_cmd(ArmModule& arm, const ArmCmd& cmd)
{
    arm.cmd_buf().write(cmd);
    // Give the control thread one loop period to pick it up
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / ARM_LOOP_HZ + 5));
}

static void print_state(const ArmState& s)
{
    std::printf("  joints: [%.3f, %.3f, %.3f, %.3f] rad\n",
                s.joints[0], s.joints[1], s.joints[2], s.joints[3]);
    std::printf("  EE pos: (%.4f, %.4f, %.4f) m\n", s.px, s.py, s.pz);
    std::printf("  reached_goal=%d  hw_ok=%d\n", (int)s.reached_goal, (int)s.hw_ok);
}

// ── stages ───────────────────────────────────────────────────────────────────
static bool stage_A(ArmModule& arm)
{
    std::printf("\n=== Stage A: init & ping ===\n");
    bool ok = arm.start();
    if (!ok) {
        std::printf("FAIL: arm.start() returned false – check port %s, baud %d, motor power\n",
                    DXL_PORT, DXL_BAUD);
        return false;
    }
    // Wait for the control thread to write its first state
    std::this_thread::sleep_for(100ms);
    ArmState s = poll_state(arm);
    if (!s.hw_ok) {
        std::printf("FAIL: hw_ok=false in state buffer\n");
        return false;
    }
    std::printf("PASS: connected. Initial state:\n");
    print_state(s);
    return true;
}

static bool stage_B(ArmModule& arm)
{
    std::printf("\n=== Stage B: move to home (all joints = 0) ===\n");
    ArmCmd cmd;
    cmd.type   = ArmCmd::Type::MoveJoint;
    cmd.joints = {0.f, 0.f, 0.f, 0.f};
    send_cmd(arm, cmd);

    if (!wait_goal(arm)) {
        std::printf("FAIL: timed out waiting for home position\n");
        std::printf("  Current state:\n");
        print_state(poll_state(arm));
        return false;
    }
    std::printf("PASS: reached home.\n");
    print_state(poll_state(arm));
    return true;
}

static bool stage_C(ArmModule& arm)
{
    std::printf("\n=== Stage C: gripper open / close ===\n");

    std::printf("  Opening gripper (Release)…\n");
    ArmCmd open_cmd;
    open_cmd.type = ArmCmd::Type::Release;
    send_cmd(arm, open_cmd);
    std::this_thread::sleep_for(1s);
    std::printf("  >> Visually confirm gripper is OPEN. Press Enter to continue.\n");
    std::getchar();

    std::printf("  Closing gripper (Grip)…\n");
    ArmCmd grip_cmd;
    grip_cmd.type = ArmCmd::Type::Grip;
    send_cmd(arm, grip_cmd);
    std::this_thread::sleep_for(1s);
    std::printf("  >> Visually confirm gripper is CLOSED. Press Enter to continue.\n");
    std::getchar();

    // Return to open for safety
    send_cmd(arm, open_cmd);
    std::this_thread::sleep_for(500ms);

    std::printf("PASS: gripper stage done (visual confirmation only).\n");
    return true;
}

static bool stage_D(ArmModule& arm)
{
    std::printf("\n=== Stage D: single-axis sweeps (±%.2f rad each) ===\n", SWEEP_RAD);

    const char* names[] = {"J1", "J2", "J3", "J4"};
    bool all_ok = true;

    for (int i = 0; i < 4; ++i) {
        // positive sweep
        std::printf("  %s +%.2f rad… ", names[i], SWEEP_RAD);
        ArmCmd cmd;
        cmd.type   = ArmCmd::Type::MoveJoint;
        cmd.joints = {0.f, 0.f, 0.f, 0.f};
        cmd.joints[i] = SWEEP_RAD;
        send_cmd(arm, cmd);
        if (!wait_goal(arm)) {
            std::printf("FAIL (timeout)\n");
            all_ok = false;
        } else {
            ArmState s = poll_state(arm);
            std::printf("OK  actual=%.3f\n", s.joints[i]);
        }

        // negative sweep
        std::printf("  %s -%.2f rad… ", names[i], SWEEP_RAD);
        cmd.joints[i] = -SWEEP_RAD;
        send_cmd(arm, cmd);
        if (!wait_goal(arm)) {
            std::printf("FAIL (timeout)\n");
            all_ok = false;
        } else {
            ArmState s = poll_state(arm);
            std::printf("OK  actual=%.3f\n", s.joints[i]);
        }

        // return to zero
        cmd.joints[i] = 0.f;
        send_cmd(arm, cmd);
        wait_goal(arm);
    }

    if (all_ok) std::printf("PASS: all axes swept.\n");
    else        std::printf("FAIL: one or more axes timed out – check IDs and wiring.\n");
    return all_ok;
}

static bool stage_E(ArmModule& arm)
{
    std::printf("\n=== Stage E: FK / IK round-trip ===\n");

    // Read the arm's current joint angles as the test input
    ArmState s = poll_state(arm);
    std::printf("  Input joints:  [%.4f, %.4f, %.4f, %.4f]\n",
                s.joints[0], s.joints[1], s.joints[2], s.joints[3]);

    Pose ee = fk(s.joints);
    std::printf("  FK result:     (%.4f, %.4f, %.4f) pitch=%.4f\n",
                ee.x, ee.y, ee.z, ee.pitch);

    auto ik_result = ik(ee);
    if (!ik_result) {
        std::printf("FAIL: IK returned nullopt for FK output – workspace or limit issue\n");
        return false;
    }

    float max_err = 0.f;
    std::printf("  IK result:     [%.4f, %.4f, %.4f, %.4f]\n",
                (*ik_result)[0], (*ik_result)[1], (*ik_result)[2], (*ik_result)[3]);
    for (int i = 0; i < 4; ++i) {
        float err = std::abs((*ik_result)[i] - s.joints[i]);
        max_err = std::max(max_err, err);
    }
    std::printf("  Max joint error: %.5f rad  (threshold: %.5f)\n",
                max_err, GOAL_REACHED_THRESH);

    if (max_err > GOAL_REACHED_THRESH) {
        std::printf("FAIL: FK/IK round-trip error exceeds threshold\n");
        return false;
    }
    std::printf("PASS: FK/IK round-trip OK.\n");
    return true;
}

static bool stage_F(ArmModule& arm)
{
    std::printf("\n=== Stage F: MovePose command ===\n");
    std::printf("  Target: (%.3f, %.3f, %.3f) pitch=%.3f\n",
                IK_TARGET_X, IK_TARGET_Y, IK_TARGET_Z, IK_TARGET_PITCH);

    // Verify the target is reachable via IK before sending
    Pose target{IK_TARGET_X, IK_TARGET_Y, IK_TARGET_Z, IK_TARGET_PITCH};
    auto ik_result = ik(target);
    if (!ik_result) {
        std::printf("FAIL: IK says target is unreachable – adjust IK_TARGET_* constants\n");
        return false;
    }
    std::printf("  Expected joints: [%.4f, %.4f, %.4f, %.4f]\n",
                (*ik_result)[0], (*ik_result)[1], (*ik_result)[2], (*ik_result)[3]);

    ArmCmd cmd;
    cmd.type  = ArmCmd::Type::MovePose;
    cmd.px    = IK_TARGET_X;
    cmd.py    = IK_TARGET_Y;
    cmd.pz    = IK_TARGET_Z;
    cmd.pitch = IK_TARGET_PITCH;
    send_cmd(arm, cmd);

    if (!wait_goal(arm)) {
        std::printf("FAIL: timed out waiting for pose target\n");
        print_state(poll_state(arm));
        return false;
    }

    ArmState s = poll_state(arm);
    std::printf("PASS: pose reached.\n");
    print_state(s);

    // Return to home
    std::printf("  Returning to home…\n");
    ArmCmd home;
    home.type   = ArmCmd::Type::MoveJoint;
    home.joints = {0.f, 0.f, 0.f, 0.f};
    send_cmd(arm, home);
    wait_goal(arm);

    return true;
}

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    const char* only = (argc > 1) ? argv[1] : nullptr;
    auto want = [&](const char* s) {
        return only == nullptr || std::strcmp(only, s) == 0;
    };

    std::printf("=== ArmModule hardware verification ===\n");
    std::printf("Port: %s  Baud: %d\n", DXL_PORT, DXL_BAUD);

    ArmModule arm;
    int failed = 0;

    // Stage A must always run first to initialise hardware
    if (!stage_A(arm)) {
        std::printf("\nHardware init failed – aborting all stages.\n");
        return 1;
    }

    if (want("B") || only == nullptr) failed += !stage_B(arm);
    if (want("C") || only == nullptr) failed += !stage_C(arm);
    if (want("D") || only == nullptr) failed += !stage_D(arm);
    if (want("E") || only == nullptr) failed += !stage_E(arm);
    if (want("F") || only == nullptr) failed += !stage_F(arm);

    // Always go home on exit
    std::printf("\n=== Press Enter to return to home and shut down ===\n");
    std::getchar();
    std::printf("Returning to home...\n");
    ArmCmd home;
    home.type   = ArmCmd::Type::MoveJoint;
    home.joints = {0.f, 0.f, 0.f, 0.f};
    send_cmd(arm, home);
    wait_goal(arm);

    arm.stop();

    std::printf("\n=== Result: %d stage(s) failed ===\n", failed);
    return failed ? 1 : 0;
}
