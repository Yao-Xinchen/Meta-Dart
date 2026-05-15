// Top-stop calibration parameter test.
//
// What this program does:
//   - Drives each hook toward the top hard stop.
//   - Uses the opposite of left_dir/right_dir because positive dir is stretch.
//   - Uses a PD velocity controller, so calibration speed is tuned separately
//     from the current safety cap.
//   - Watches velocity while the hook presses into the stop.
//   - If velocity stays below jam_vel for jam_time_s, it accepts that position
//     as top/home/zero.
//   - Prints the zero position and the travel-to-top observed during the test.
//   - Sends zero current at the end.
//
// Use this after direction signs are correct.
//
// What to check:
//   - Hooks should move gently toward the top stop, not toward stretch.
//   - It should not declare success before the hook actually reaches the stop.
//   - It should not grind at the stop for too long.
//   - If it moves too fast, lower target_vel before raising/lowering max_current.
//   - If it times out, target_vel/current cap may be too low or jam_vel/jam_time too strict.
//
// Params this helps tune:
//   - SPRING_STRETCHER_CALIB_VEL_RAD_S
//   - SPRING_STRETCHER_CALIB_MAX_CURRENT_A
//   - SPRING_STRETCHER_CALIB_VEL_KP / KD
//   - SPRING_STRETCHER_CALIB_JAM_VEL_RAD_S
//   - SPRING_STRETCHER_CALIB_JAM_TIME_S
//   - SPRING_STRETCHER_CALIB_TIMEOUT_S
//
//   sudo ./build/test_stretcher_calibration [target_vel] [max_current] [vel_kp] [vel_kd] [jam_vel] [jam_time_s] [timeout_s] [left_dir] [right_dir] [can_iface]

#include "stretcher_test_utils.hpp"

#include <cstdlib>

int main(int argc, char** argv)
{
    const float target_vel = (argc > 1) ? std::strtof(argv[1], nullptr) : SPRING_STRETCHER_CALIB_VEL_RAD_S;
    const float max_current = (argc > 2) ? std::strtof(argv[2], nullptr) : SPRING_STRETCHER_CALIB_MAX_CURRENT_A;
    const float vel_kp = (argc > 3) ? std::strtof(argv[3], nullptr) : SPRING_STRETCHER_CALIB_VEL_KP;
    const float vel_kd = (argc > 4) ? std::strtof(argv[4], nullptr) : SPRING_STRETCHER_CALIB_VEL_KD;
    const float jam_vel = (argc > 5) ? std::strtof(argv[5], nullptr) : SPRING_STRETCHER_CALIB_JAM_VEL_RAD_S;
    const float jam_time_s = (argc > 6) ? std::strtof(argv[6], nullptr) : SPRING_STRETCHER_CALIB_JAM_TIME_S;
    const float timeout_s = (argc > 7) ? std::strtof(argv[7], nullptr) : SPRING_STRETCHER_CALIB_TIMEOUT_S;
    const float left_dir = (argc > 8) ? std::strtof(argv[8], nullptr) : SPRING_STRETCHER_LEFT_DIR;
    const float right_dir = (argc > 9) ? std::strtof(argv[9], nullptr) : SPRING_STRETCHER_RIGHT_DIR;
    const char* iface = (argc > 10) ? argv[10] : SPRING_STRETCHER_CAN_IFACE;

    std::printf("\n=== Spring stretcher calibration test ===\n");
    std::printf("Purpose: tune top-stop zeroing by detecting when hooks get stuck.\n");
    std::printf("Motor command: PD velocity control toward top stop, then zero current.\n");
    std::printf("Expected: each motor prints \"top stop\" only after physically reaching top.\n");
    std::printf("Args: target_vel=%.2f rad/s max_current=%.2f A vel_kp=%.2f vel_kd=%.2f\n",
                target_vel, max_current, vel_kp, vel_kd);
    std::printf("Jam: jam_vel=%.2f rad/s jam_time=%.2fs timeout=%.2fs\n",
                jam_vel, jam_time_s, timeout_s);
    std::printf("Dirs: left=%.1f right=%.1f iface=%s\n\n", left_dir, right_dir, iface);

    stretcher_test::M3508Bus bus;
    if (!bus.open(iface)) return 1;

    std::array<stretcher_test::Feedback, 2> fb{};
    fb[0].id = SPRING_STRETCHER_LEFT_ID;
    fb[1].id = SPRING_STRETCHER_RIGHT_ID;

    const auto result = stretcher_test::calibrate_top_stops(
        bus, fb, target_vel, max_current, vel_kp, vel_kd,
        jam_vel, jam_time_s, timeout_s, left_dir, right_dir);

    std::printf("result=%s left_zero=%.3f right_zero=%.3f\n",
                result.ok ? "OK" : "FAIL", result.zero[0], result.zero[1]);
    return result.ok ? 0 : 1;
}
