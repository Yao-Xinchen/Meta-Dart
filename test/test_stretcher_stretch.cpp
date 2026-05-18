// Calibrate, stretch to a target length, hold, then retract.
//
// What this program does:
//   - First runs the same top-stop calibration as test_stretcher_calibration.
//   - Treats that top stop as position 0.
//   - Runs a simple position-to-velocity-to-current controller to move both
//     output shafts to stretch_rad away from the top stop.
//   - Holds there for hold_s seconds.
//   - Retracts both hooks back to home/zero.
//   - Prints position error and current during stretch/retract.
//
// Use this only after monitor, direction, and calibration are working.
//
// What to check:
//   - Stretch motion should be smooth and symmetric.
//   - Current should not sit at max_current for the whole move unless the spring
//     is genuinely too hard to pull.
//   - It should settle near target without oscillating badly.
//   - Retract should return close to 0.
//
// Params this helps tune:
//   - SPRING_STRETCHER_STRETCH_RAD
//   - SPRING_STRETCHER_MAX_CURRENT_A
//   - SPRING_STRETCHER_POS_KP
//   - SPRING_STRETCHER_VEL_KP
//   - SPRING_STRETCHER_MAX_VEL_RAD_S
//   - SPRING_STRETCHER_HOLD_TIME_S
//
//   sudo ./build/test_stretcher_stretch [stretch_rad] [max_current] [pos_kp] [vel_kp] [max_vel] [hold_s] [can_iface]

#include "stretcher_test_utils.hpp"

#include <chrono>
#include <cstdlib>
#include <thread>

static bool run_to_target(stretcher_test::M3508Bus& bus,
                          std::array<stretcher_test::Feedback, 2>& fb,
                          const std::array<float, 2>& zero,
                          float target_rad,
                          float max_current,
                          float pos_kp,
                          float vel_kp,
                          float max_vel,
                          float timeout_s)
{
    using Clock = std::chrono::steady_clock;
    using namespace std::chrono_literals;

    const auto t0 = Clock::now();
    auto next_print = t0;

    while (std::chrono::duration<float>(Clock::now() - t0).count() < timeout_s) {
        bus.poll(fb);

        const float pos_l = fb[0].raw_position - zero[0];
        const float pos_r = fb[1].raw_position - zero[1];
        const float target_l = SPRING_STRETCHER_LEFT_DIR * target_rad;
        const float target_r = SPRING_STRETCHER_RIGHT_DIR * target_rad;
        const float vel_goal_l = stretcher_test::clamp(pos_kp * (target_l - pos_l), max_vel);
        const float vel_goal_r = stretcher_test::clamp(pos_kp * (target_r - pos_r), max_vel);
        const float cur_l = stretcher_test::clamp(vel_kp * (vel_goal_l - fb[0].velocity), max_current);
        const float cur_r = stretcher_test::clamp(vel_kp * (vel_goal_r - fb[1].velocity), max_current);

        bus.send(cur_l, cur_r, fb[0].id, fb[1].id);

        if (Clock::now() >= next_print) {
            std::printf("target=%.3f | left pos=%7.3f err=%7.3f cur=%6.2f | "
                        "right pos=%7.3f err=%7.3f cur=%6.2f\n",
                        target_rad,
                        pos_l, target_l - pos_l, cur_l,
                        pos_r, target_r - pos_r, cur_r);
            next_print += 100ms;
        }

        const bool left_ok = std::fabs(target_l - pos_l) <= SPRING_STRETCHER_POS_TOL_RAD &&
                             std::fabs(fb[0].velocity) <= SPRING_STRETCHER_VEL_TOL_RAD_S;
        const bool right_ok = std::fabs(target_r - pos_r) <= SPRING_STRETCHER_POS_TOL_RAD &&
                              std::fabs(fb[1].velocity) <= SPRING_STRETCHER_VEL_TOL_RAD_S;
        if (left_ok && right_ok) {
            bus.send(0.f, 0.f, fb[0].id, fb[1].id);
            return true;
        }

        std::this_thread::sleep_for(2ms);
    }

    bus.send(0.f, 0.f, fb[0].id, fb[1].id);
    return false;
}

int main(int argc, char** argv)
{
    using namespace std::chrono_literals;

    const float stretch_rad = (argc > 1) ? std::strtof(argv[1], nullptr) : SPRING_STRETCHER_STRETCH_RAD;
    const float max_current = (argc > 2) ? std::strtof(argv[2], nullptr) : SPRING_STRETCHER_MAX_CURRENT_A;
    const float pos_kp = (argc > 3) ? std::strtof(argv[3], nullptr) : SPRING_STRETCHER_POS_KP;
    const float vel_kp = (argc > 4) ? std::strtof(argv[4], nullptr) : SPRING_STRETCHER_VEL_KP;
    const float max_vel = (argc > 5) ? std::strtof(argv[5], nullptr) : SPRING_STRETCHER_MAX_VEL_RAD_S;
    const float hold_s = (argc > 6) ? std::strtof(argv[6], nullptr) : SPRING_STRETCHER_HOLD_TIME_S;
    const char* iface = (argc > 7) ? argv[7] : SPRING_STRETCHER_CAN_IFACE;

    std::printf("\n=== Spring stretcher stretch-cycle test ===\n");
    std::printf("Purpose: tune actual spring stretch motion after calibration works.\n");
    std::printf("Sequence: calibrate top -> stretch -> hold -> retract home.\n");
    std::printf("Expected: smooth symmetric motion, no violent oscillation, returns near zero.\n");
    std::printf("Args: stretch=%.3f rad max_current=%.2f A pos_kp=%.2f vel_kp=%.2f max_vel=%.2f hold=%.2fs\n",
                stretch_rad, max_current, pos_kp, vel_kp, max_vel, hold_s);
    std::printf("Retract uses the same max_vel and max_current. iface=%s\n\n", iface);

    stretcher_test::M3508Bus bus;
    if (!bus.open(iface)) return 1;

    std::array<stretcher_test::Feedback, 2> fb{};
    fb[0].id = SPRING_STRETCHER_LEFT_ID;
    fb[1].id = SPRING_STRETCHER_RIGHT_ID;

    const auto calib = stretcher_test::calibrate_top_stops(
        bus, fb,
        SPRING_STRETCHER_CALIB_VEL_RAD_S,
        SPRING_STRETCHER_CALIB_MAX_CURRENT_A,
        SPRING_STRETCHER_CALIB_VEL_KP,
        SPRING_STRETCHER_CALIB_VEL_KD,
        SPRING_STRETCHER_CALIB_JAM_VEL_RAD_S,
        SPRING_STRETCHER_CALIB_JAM_TIME_S,
        SPRING_STRETCHER_CALIB_TIMEOUT_S,
        SPRING_STRETCHER_LEFT_DIR,
        SPRING_STRETCHER_RIGHT_DIR);
    if (!calib.ok) return 1;

    std::printf("stretching to %.3f rad\n", stretch_rad);
    if (!run_to_target(bus, fb, calib.zero, stretch_rad, max_current, pos_kp, vel_kp, max_vel, 6.0f)) {
        std::fprintf(stderr, "stretch target timed out\n");
        return 1;
    }

    std::printf("holding for %.2fs\n", hold_s);
    std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<float>(hold_s)));

    std::printf("retracting to home\n");
    if (!run_to_target(bus, fb, calib.zero, 0.f, max_current, pos_kp, vel_kp, max_vel, 12.0f)) {
        std::fprintf(stderr, "retract target timed out\n");
        return 1;
    }

    return 0;
}
