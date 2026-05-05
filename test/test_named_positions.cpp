// test/test_named_positions.cpp
//
// Interactive named-position inspector.
//
// It moves to one named position at a time, then pauses so the operator can
// observe where that waypoint is in the real workspace.
//
// Usage:
//   ./build/test_named_positions [positions.xml] [name1 name2 ...]
//
// If no names are given, the default pickup trajectory points are inspected.

#include "arm.hpp"
#include "config.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

static constexpr float MIN_MOVE_DURATION = 2.5f;
static constexpr float MAX_MOVE_DURATION = 12.0f;
static constexpr float JOINT_CRUISE_SPEED_RAD_S = 0.35f;
static constexpr auto  MOVE_TIMEOUT = 25s;

static const std::vector<std::string> DEFAULT_POINTS = {
    "home",
    "forward_prep",
    "backward_prep",
    "s0_prep",
    "s0_grasp",
    "s0_pulled_out",
    "backward_prep",
    "forward_prep",
    "loading",
    "home",
};

static std::map<std::string, std::array<float, 4>> load_xml(const char* path)
{
    std::map<std::string, std::array<float, 4>> positions;
    std::ifstream f(path);
    if (!f) {
        std::fprintf(stderr, "ERROR: cannot open %s\n", path);
        return positions;
    }

    auto get_attr = [](const std::string& line, const std::string& attr) -> std::string {
        auto pos = line.find(attr + "=\"");
        if (pos == std::string::npos) return {};
        pos += attr.size() + 2;
        auto end = line.find('"', pos);
        return line.substr(pos, end - pos);
    };

    std::string line;
    while (std::getline(f, line)) {
        if (line.find("<position") == std::string::npos) continue;
        const std::string name = get_attr(line, "name");
        if (name.empty()) continue;

        std::array<float, 4> joints{};
        joints[0] = std::stof(get_attr(line, "j1"));
        joints[1] = std::stof(get_attr(line, "j2"));
        joints[2] = std::stof(get_attr(line, "j3"));
        joints[3] = std::stof(get_attr(line, "j4"));
        positions[name] = joints;
    }
    return positions;
}

static ArmState poll_state(ArmModule& arm)
{
    static ArmState last{};
    arm.read_state(last);
    return last;
}

static float max_joint_delta(const std::array<float, 4>& from,
                             const std::array<float, 4>& to)
{
    float max_delta = 0.f;
    for (int i = 0; i < 4; ++i)
        max_delta = std::max(max_delta, std::abs(to[i] - from[i]));
    return max_delta;
}

static float clampf(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static float duration_for_segment(const std::array<float, 4>& from,
                                  const std::array<float, 4>& to)
{
    const float max_delta = max_joint_delta(from, to);
    return clampf(max_delta / JOINT_CRUISE_SPEED_RAD_S,
                  MIN_MOVE_DURATION, MAX_MOVE_DURATION);
}

static bool wait_goal(ArmModule& arm)
{
    using Clock = std::chrono::steady_clock;
    const auto deadline = Clock::now() + MOVE_TIMEOUT;
    while (Clock::now() < deadline) {
        const ArmState state = poll_state(arm);
        if (!state.hw_ok)
            return false;
        if (state.reached_goal)
            return true;
        std::this_thread::sleep_for(50ms);
    }
    return false;
}

static void print_joints(const char* label, const std::array<float, 4>& joints)
{
    std::printf("  %-10s [%.4f, %.4f, %.4f, %.4f]\n",
                label, joints[0], joints[1], joints[2], joints[3]);
}

static bool ask_continue(const char* prompt)
{
    std::printf("%s", prompt);
    std::fflush(stdout);

    std::string line;
    if (!std::getline(std::cin, line))
        return false;
    return !(line == "q" || line == "Q");
}

int main(int argc, char* argv[])
{
    const char* xml_path = "positions.xml";
    std::vector<std::string> names;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.size() > 4 && arg.substr(arg.size() - 4) == ".xml")
            xml_path = argv[i];
        else
            names.push_back(arg);
    }
    if (names.empty())
        names = DEFAULT_POINTS;

    const auto positions = load_xml(xml_path);
    if (positions.empty())
        return 1;

    std::printf("=== Named position inspector ===\n");
    std::printf("Positions file: %s (%zu entries)\n", xml_path, positions.size());
    std::printf("Move speed: %.2f rad/s equivalent, clamped to %.1f-%.1fs\n",
                JOINT_CRUISE_SPEED_RAD_S, MIN_MOVE_DURATION, MAX_MOVE_DURATION);
    std::printf("Press Enter to move, q + Enter to quit.\n\n");

    for (const auto& name : names) {
        if (!positions.count(name)) {
            std::fprintf(stderr, "ERROR: '%s' not found in %s\n", name.c_str(), xml_path);
            return 1;
        }
    }

    ArmModule arm;
    if (!arm.start()) {
        std::fprintf(stderr, "ERROR: arm.start() failed\n");
        return 1;
    }
    std::this_thread::sleep_for(100ms);
    if (!poll_state(arm).hw_ok) {
        std::fprintf(stderr, "ERROR: hw_ok=false\n");
        arm.stop();
        return 1;
    }

    for (size_t i = 0; i < names.size(); ++i) {
        const auto& name = names[i];
        const auto& target = positions.at(name);

        std::printf("[%zu/%zu] %s\n", i + 1, names.size(), name.c_str());
        print_joints("target:", target);
        print_joints("current:", poll_state(arm).joints);
        if (!ask_continue("  Press Enter to move to this point, q to quit: "))
            break;

        const float duration = duration_for_segment(poll_state(arm).joints, target);
        std::printf("  moving for %.2fs...\n", duration);
        arm.move_joints(target, duration);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / ARM_LOOP_HZ + 5));

        if (!wait_goal(arm)) {
            std::fprintf(stderr, "ERROR: timed out at '%s'\n", name.c_str());
            print_joints("actual:", poll_state(arm).joints);
            arm.stop();
            return 1;
        }

        print_joints("actual:", poll_state(arm).joints);
        std::printf("  Observe and label the real-world pose now.\n");
        if (!ask_continue("  Press Enter for next point, q to quit: "))
            break;
        std::printf("\n");
    }

    std::printf("\nDone. Press Enter to power off torque...");
    std::fflush(stdout);
    std::string line;
    std::getline(std::cin, line);
    arm.stop();
    return 0;
}
