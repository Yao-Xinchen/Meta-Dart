// Passive CAN feedback monitor for the two DJI M3508 spring stretcher motors.
//
// What this program does:
//   - Opens the configured SocketCAN interface.
//   - Listens for feedback frames from the left/right DJI M3508 motors.
//   - Sends zero current commands, so the motors should not intentionally move.
//   - Prints raw cumulative position, velocity, current, and whether feedback
//     has been seen from each motor.
//
// Use this first, before moving anything.
//
// What to check:
//   - seen=1 for both motors means the CAN IDs and interface are correct.
//   - Turning a hook by hand should change that motor's pos value.
//   - Velocity should be near 0 when stationary.
//   - Current should be near 0 when no load is applied.
//
// Params this helps verify:
//   - SPRING_STRETCHER_CAN_IFACE
//   - SPRING_STRETCHER_LEFT_ID
//   - SPRING_STRETCHER_RIGHT_ID
//
//   sudo ./build/test_stretcher_monitor [seconds] [can_iface]

#include "stretcher_test_utils.hpp"

#include <chrono>
#include <cstdlib>
#include <thread>

int main(int argc, char** argv)
{
    using namespace std::chrono_literals;
    using Clock = std::chrono::steady_clock;

    const float seconds = (argc > 1) ? std::strtof(argv[1], nullptr) : 10.0f;
    const char* iface = (argc > 2) ? argv[2] : SPRING_STRETCHER_CAN_IFACE;

    std::printf("\n=== Spring stretcher monitor ===\n");
    std::printf("Purpose: verify CAN interface, motor IDs, and passive feedback.\n");
    std::printf("Motor command: zero current only; motors should not move by themselves.\n");
    std::printf("Expected: seen=1 for both motors; hand motion changes pos.\n");
    std::printf("Running for %.2fs on %s\n\n", seconds, iface);

    stretcher_test::M3508Bus bus;
    if (!bus.open(iface)) return 1;

    std::array<stretcher_test::Feedback, 2> fb{};
    fb[0].id = SPRING_STRETCHER_LEFT_ID;
    fb[1].id = SPRING_STRETCHER_RIGHT_ID;

    const auto end = Clock::now() + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<float>(seconds));
    auto next_print = Clock::now();

    while (Clock::now() < end) {
        bus.poll(fb);
        bus.send(0.f, 0.f, fb[0].id, fb[1].id);

        if (Clock::now() >= next_print) {
            stretcher_test::print_feedback(fb);
            next_print += 100ms;
        }
        std::this_thread::sleep_for(2ms);
    }

    bus.send(0.f, 0.f, fb[0].id, fb[1].id);
    return 0;
}
