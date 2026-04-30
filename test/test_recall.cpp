// test/test_recall.cpp
//
// Record-and-recall test:
//   1. User positions arm at A by hand (torque is off before init).
//   2. Press Enter — arm initialises and holds A.
//   3. Arm moves to B (hardcoded).
//   4. Press Enter — arm returns to recorded A.
//
// Run as: sudo ./test_recall

#include "arm.hpp"
#include "config.hpp"
#include "types.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

using namespace std::chrono_literals;

// ── tunables ──────────────────────────────────────────────────────────────────
static constexpr std::array<float, 4> JOINTS_B   = {0.f, 0.00f, 0.00f, 0.00f};  // TODO
static constexpr float                MOVE_DURATION = 3.0f;  // seconds per move
static constexpr auto                 MOVE_TIMEOUT  = 10s;   // max wait for reached_goal

// ── helpers ───────────────────────────────────────────────────────────────────
static ArmState poll_state(ArmModule& arm)
{
    static ArmState last{};
    arm.state_buf().read(last);
    return last;
}

static bool wait_goal(ArmModule& arm)
{
    using Clock = std::chrono::steady_clock;
    auto deadline = Clock::now() + MOVE_TIMEOUT;

    // Wait until the arm loop picks up the command and clears reached_goal.
    // Without this, a stale reached_goal=true from the previous move causes
    // an immediate false return before the arm has started moving.
    while (Clock::now() < deadline) {
        if (!poll_state(arm).reached_goal) break;
        std::this_thread::sleep_for(20ms);
    }

    while (Clock::now() < deadline) {
        if (poll_state(arm).reached_goal) return true;
        std::this_thread::sleep_for(50ms);
    }
    return false;
}

static void send_joints(ArmModule& arm, const std::array<float, 4>& joints, float duration)
{
    ArmCmd cmd;
    cmd.type     = ArmCmd::Type::MoveJoint;
    cmd.joints   = joints;
    cmd.duration = duration;
    arm.cmd_buf().write(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / ARM_LOOP_HZ + 5));
}

static void print_joints(const char* label, const std::array<float, 4>& j)
{
    std::printf("  %s: [%.3f, %.3f, %.3f, %.3f] rad\n", label, j[0], j[1], j[2], j[3]);
}

// ── main ──────────────────────────────────────────────────────────────────────
int main()
{
    std::printf("=== Record-and-recall test ===\n");
    std::printf("Port: %s  Baud: %d\n\n", DXL_PORT, DXL_BAUD);

    // ── Record A ──────────────────────────────────────────────────────────────
    // Torque is off until arm.start() is called — move the arm freely now.
    std::printf("Move arm to position A by hand, then press Enter...\n");
    std::getchar();

    ArmModule arm;
    if (!arm.start()) {
        std::fprintf(stderr, "ERROR: arm.start() failed — check port and motor power\n");
        return 1;
    }
    // Give the control thread one tick to publish the first state.
    std::this_thread::sleep_for(100ms);

    ArmState s = poll_state(arm);
    if (!s.hw_ok) {
        std::fprintf(stderr, "ERROR: hw_ok=false after init\n");
        arm.stop();
        return 1;
    }

    // arm.start() reads current motor positions before enabling torque, so
    // state.joints reflects exactly where the user left the arm.
    std::array<float, 4> pos_a = s.joints;
    std::printf("Position A recorded:\n");
    print_joints("A", pos_a);

    // ── Move to B ─────────────────────────────────────────────────────────────
    std::printf("\nMoving to B...\n");
    print_joints("B", JOINTS_B);
    send_joints(arm, JOINTS_B, MOVE_DURATION);

    if (!wait_goal(arm)) {
        std::fprintf(stderr, "ERROR: timed out reaching B\n");
        arm.stop();
        return 1;
    }
    std::printf("Reached B.\n");

    // ── Recall A ──────────────────────────────────────────────────────────────
    std::printf("\nPress Enter to return to A...\n");
    std::getchar();

    std::printf("Returning to A...\n");
    send_joints(arm, pos_a, MOVE_DURATION);

    if (!wait_goal(arm)) {
        std::fprintf(stderr, "ERROR: timed out returning to A\n");
        arm.stop();
        return 1;
    }
    std::printf("Reached A.\n");
    print_joints("current", poll_state(arm).joints);

    std::printf("\nPress Enter to power off the arm...\n");
    std::getchar();

    arm.stop();
    std::printf("Done.\n");
    return 0;
}
