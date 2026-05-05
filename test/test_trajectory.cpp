// test/test_trajectory.cpp
//
// Move the arm through a named sequence of positions loaded from an XML file.
//
// Usage:
//   sudo ./test_trajectory [positions.xml] [name1 name2 ...]
//
// If no names are given, the default trajectory is used.
// If no XML path is given, positions.xml in the current directory is used.

#include "arm.hpp"
#include "config.hpp"
#include "types.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

// ── tunables ──────────────────────────────────────────────────────────────────
static constexpr float MIN_MOVE_DURATION = 1.5f;
static constexpr float MAX_MOVE_DURATION = 7.0f;
static constexpr float JOINT_CRUISE_SPEED_RAD_S = 0.60f;
static constexpr auto  MOVE_TIMEOUT  = 15s;

static const std::vector<std::string> DEFAULT_TRAJECTORY = {
    "home", "forward_prep", "backward_prep",
    "s0_prep", "s0_grasp", "s0_pulled_out",
    "backward_prep", "forward_prep", "loading", "home",
};

// ── XML loader ────────────────────────────────────────────────────────────────
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
        std::string name = get_attr(line, "name");
        if (name.empty()) continue;
        std::array<float, 4> j{};
        j[0] = std::stof(get_attr(line, "j1"));
        j[1] = std::stof(get_attr(line, "j2"));
        j[2] = std::stof(get_attr(line, "j3"));
        j[3] = std::stof(get_attr(line, "j4"));
        positions[name] = j;
    }
    return positions;
}

// ── helpers ───────────────────────────────────────────────────────────────────
static ArmState poll_state(ArmModule& arm)
{
    static ArmState last{};
    arm.read_state(last);
    return last;
}

static bool wait_goal(ArmModule& arm)
{
    using Clock = std::chrono::steady_clock;
    auto deadline = Clock::now() + MOVE_TIMEOUT;
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
    arm.move_joints(joints, duration);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / ARM_LOOP_HZ + 5));
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
    float max_delta = 0.f;
    for (int i = 0; i < 4; ++i)
        max_delta = std::max(max_delta, std::abs(to[i] - from[i]));
    return clampf(max_delta / JOINT_CRUISE_SPEED_RAD_S,
                  MIN_MOVE_DURATION, MAX_MOVE_DURATION);
}

static void print_joints(const char* label, const std::array<float, 4>& j)
{
    std::printf("  %-10s [%.3f, %.3f, %.3f, %.3f]\n", label, j[0], j[1], j[2], j[3]);
}

static void print_joint_errors(const std::array<float, 4>& actual,
                               const std::array<float, 4>& target)
{
    const char* names[4] = {"J1", "J2", "J3", "J4"};
    for (int i = 0; i < 4; ++i) {
        float err = actual[i] - target[i];
        if (std::abs(err) > GOAL_REACHED_THRESH)
            std::printf("  %s: got %.3f, wanted %.3f  (error %.3f rad) ← MISSED\n",
                        names[i], actual[i], target[i], err);
    }
}

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    const char* xml_path = "positions.xml";
    std::vector<std::string> trajectory;

    // Parse arguments: first non-flag arg ending in .xml is the file path,
    // remaining args are position names.
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a.size() > 4 && a.substr(a.size() - 4) == ".xml")
            xml_path = argv[i];
        else
            trajectory.push_back(a);
    }
    if (trajectory.empty())
        trajectory = DEFAULT_TRAJECTORY;

    // Load positions
    auto positions = load_xml(xml_path);
    if (positions.empty()) return 1;

    std::printf("=== Trajectory test ===\n");
    std::printf("Positions file: %s  (%zu entries)\n", xml_path, positions.size());
    std::printf("Trajectory (%zu steps):\n", trajectory.size());
    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& name = trajectory[i];
        bool found = positions.count(name) > 0;
        std::printf("  %zu. %s%s\n", i + 1, name.c_str(), found ? "" : "  ← NOT FOUND");
        if (!found) {
            std::fprintf(stderr, "ERROR: '%s' not in %s\n", name.c_str(), xml_path);
            return 1;
        }
    }
    std::printf("\n");

    // Init arm
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

    // Execute trajectory
    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& name   = trajectory[i];
        const auto& target = positions.at(name);

        std::printf("[%zu/%zu] → %s\n", i + 1, trajectory.size(), name.c_str());
        print_joints("target:", target);
        auto current = poll_state(arm).joints;
        print_joints("current:", current);

        const float duration = duration_for_segment(current, target);
        std::printf("  duration:  %.2fs\n", duration);
        send_joints(arm, target, duration);
        if (!wait_goal(arm)) {
            auto stuck = poll_state(arm).joints;
            std::fprintf(stderr, "ERROR: timed out at step '%s'\n", name.c_str());
            print_joints("stuck at:", stuck);
            print_joints("target was:", target);
            print_joint_errors(stuck, target);
            arm.stop();
            return 1;
        }
        print_joints("arrived:", poll_state(arm).joints);
        std::printf("\n");
    }

    std::printf("\nTrajectory complete. Press Enter to power off...\n");
    std::getchar();

    arm.stop();
    return 0;
}
