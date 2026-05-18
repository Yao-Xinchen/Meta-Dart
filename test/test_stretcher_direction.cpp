// Fixed-current direction test.
//
// Positive current uses left_dir/right_dir. If the config directions are right,
// hooks should move in the stretch direction.
//
// What this program does:
//   - Waits until both motors publish feedback.
//   - Sends a small fixed current for a short time.
//   - Uses left_dir and right_dir to decide the sign of the current.
//   - Prints the output shaft position delta after the move.
//   - Sends zero current at the end.
//
// Use this after test_stretcher_monitor works.
//
// What to check:
//   - Both hooks should move in the direction that stretches the springs.
//   - If one side moves toward the top stop instead, flip that side's dir.
//   - If the wrong motor moves, fix LEFT_ID/RIGHT_ID.
//   - Start with a small current and short time.
//
// Params this helps tune:
//   - SPRING_STRETCHER_LEFT_DIR
//   - SPRING_STRETCHER_RIGHT_DIR
//   - SPRING_STRETCHER_LEFT_ID / RIGHT_ID
//
//   sudo ./build/test_stretcher_direction [current_a] [seconds] [left_dir] [right_dir] [can_iface]

#include "stretcher_test_utils.hpp"

#include <chrono>
#include <cstdlib>
#include <thread>

int main(int argc, char** argv)
{
    using namespace std::chrono_literals;
    using Clock = std::chrono::steady_clock;

    const float current_a = (argc > 1) ? std::strtof(argv[1], nullptr) : 1.0f;
    const float seconds = (argc > 2) ? std::strtof(argv[2], nullptr) : 1.0f;
    const float left_dir = (argc > 3) ? std::strtof(argv[3], nullptr) : SPRING_STRETCHER_LEFT_DIR;
    const float right_dir = (argc > 4) ? std::strtof(argv[4], nullptr) : SPRING_STRETCHER_RIGHT_DIR;
    const char* iface = (argc > 5) ? argv[5] : SPRING_STRETCHER_CAN_IFACE;

    std::printf("\n=== Spring stretcher direction test ===\n");
    std::printf("Purpose: verify direction signs for stretch motion.\n");
    std::printf("Motor command: fixed current for a short time, then zero current.\n");
    std::printf("Expected: both hooks move in the spring-stretching direction.\n");
    std::printf("Args: current=%.2f A seconds=%.2f left_dir=%.1f right_dir=%.1f iface=%s\n\n",
                current_a, seconds, left_dir, right_dir, iface);

    stretcher_test::M3508Bus bus;
    if (!bus.open(iface)) return 1;

    std::array<stretcher_test::Feedback, 2> fb{};
    fb[0].id = SPRING_STRETCHER_LEFT_ID;
    fb[1].id = SPRING_STRETCHER_RIGHT_ID;

    while (!fb[0].seen || !fb[1].seen) {
        bus.poll(fb);
        bus.send(0.f, 0.f, fb[0].id, fb[1].id);
        std::this_thread::sleep_for(2ms);
    }

    const float left_start = fb[0].raw_position;
    const float right_start = fb[1].raw_position;
    const auto end = Clock::now() + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<float>(seconds));
    auto next_print = Clock::now();

    std::printf("sending left=%.2f A, right=%.2f A for %.2fs\n",
                left_dir * current_a, right_dir * current_a, seconds);

    while (Clock::now() < end) {
        bus.poll(fb);
        bus.send(left_dir * current_a, right_dir * current_a, fb[0].id, fb[1].id);

        if (Clock::now() >= next_print) {
            stretcher_test::print_feedback(fb);
            next_print += 100ms;
        }
        std::this_thread::sleep_for(2ms);
    }

    bus.send(0.f, 0.f, fb[0].id, fb[1].id);
    std::printf("delta: left=%.3f rad, right=%.3f rad\n",
                fb[0].raw_position - left_start, fb[1].raw_position - right_start);
    return 0;
}
