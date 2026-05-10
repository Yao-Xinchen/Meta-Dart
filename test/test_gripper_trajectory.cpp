// test/test_gripper_trajectory.cpp
//
// Stream the arm through the dart pickup trajectory and operate the gripper:
//   - open the gripper after arriving at s0_prep
//   - close the gripper after arriving at s0_grasp
//   - keep it closed for the rest of the trajectory
//
// Ordinary trajectory points are treated as fly-through waypoints. The arm only
// stops at gripper action points and at the final target.
//
// Usage:
//   ./test_gripper_trajectory [positions.xml] [name1 name2 ...]
//
// If no names are given, the default trajectory is used.
// If no XML path is given, positions.xml in the current directory is used.

#include "arm.hpp"
#include "config.hpp"
#include "types.hpp"

#include <algorithm>
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

static constexpr float MIN_STREAM_DURATION = 0.50f;
static constexpr float JOINT_CRUISE_SPEED_RAD_S = 0.60f;
static constexpr auto  STREAM_PERIOD = 20ms;
static constexpr auto  MOVE_TIMEOUT = 20s;
static constexpr auto  GRIPPER_SETTLE = 1s;

static const std::vector<std::string> DEFAULT_TRAJECTORY = {
    "home", "forward_prep", "backward_prep",
    "s0_prep", "s0_grasp", "s0_pulled_out",
    "backward_prep", "forward_prep", "loading", "home",
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

static bool wait_near(ArmModule& arm, const std::array<float, 4>& target, float threshold)
{
    using Clock = std::chrono::steady_clock;
    auto deadline = Clock::now() + MOVE_TIMEOUT;
    while (Clock::now() < deadline) {
        const ArmState state = poll_state(arm);
        if (!state.hw_ok)
            return false;
        if (max_joint_delta(state.joints, target) <= threshold)
            return true;
        std::this_thread::sleep_for(40ms);
    }
    return false;
}

static float clampf(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static float smoothstep_quintic(float t)
{
    t = clampf(t, 0.f, 1.f);
    return t * t * t * (t * (t * 6.f - 15.f) + 10.f);
}

static std::array<float, 4> lerp_joints(const std::array<float, 4>& from,
                                        const std::array<float, 4>& to,
                                        float t)
{
    std::array<float, 4> out{};
    t = clampf(t, 0.f, 1.f);
    for (int i = 0; i < 4; ++i)
        out[i] = from[i] + (to[i] - from[i]) * t;
    return out;
}

static std::vector<float> segment_lengths(const std::vector<std::array<float, 4>>& points)
{
    std::vector<float> lengths;
    if (points.size() < 2)
        return lengths;

    lengths.reserve(points.size() - 1);
    for (size_t i = 1; i < points.size(); ++i)
        lengths.push_back(max_joint_delta(points[i - 1], points[i]));
    return lengths;
}

static float total_length(const std::vector<float>& lengths)
{
    float total = 0.f;
    for (float length : lengths)
        total += length;
    return total;
}

static std::array<float, 4> sample_path_at(const std::vector<std::array<float, 4>>& points,
                                           const std::vector<float>& lengths,
                                           float distance)
{
    if (points.empty())
        return {};
    if (points.size() == 1)
        return points.front();

    float walked = 0.f;
    for (size_t i = 1; i < points.size(); ++i) {
        const float length = lengths[i - 1];
        if (length <= 1e-5f)
            continue;

        if (distance <= walked + length) {
            const float local_t = (distance - walked) / length;
            return lerp_joints(points[i - 1], points[i], local_t);
        }
        walked += length;
    }

    return points.back();
}

static float duration_for_path(const std::vector<float>& lengths)
{
    const float length = total_length(lengths);
    if (length <= 1e-5f)
        return MIN_STREAM_DURATION;
    return std::max(MIN_STREAM_DURATION, length / JOINT_CRUISE_SPEED_RAD_S);
}

static bool stream_path(ArmModule& arm,
                        const std::vector<std::array<float, 4>>& points,
                        float* duration_out)
{
    using Clock = std::chrono::steady_clock;

    if (points.empty())
        return false;

    const auto lengths = segment_lengths(points);
    const float length = total_length(lengths);
    const float duration = duration_for_path(lengths);
    if (duration_out)
        *duration_out = duration;

    const auto t0 = Clock::now();
    auto next_tick = t0;

    while (true) {
        const float elapsed = std::chrono::duration<float>(Clock::now() - t0).count();
        const float t = clampf(elapsed / duration, 0.f, 1.f);
        const float eased_distance = length * smoothstep_quintic(t);
        arm.servo_joints(sample_path_at(points, lengths, eased_distance));

        if (t >= 1.f)
            break;

        next_tick += STREAM_PERIOD;
        std::this_thread::sleep_until(next_tick);
    }

    arm.servo_joints(points.back());
    return wait_near(arm, points.back(), GOAL_REACHED_THRESH);
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
            std::printf("  %s: got %.3f, wanted %.3f  (error %.3f rad) MISSED\n",
                        names[i], actual[i], target[i], err);
    }
}

static void open_gripper(ArmModule& arm)
{
    std::printf("  gripper: open\n");
    arm.release();
    std::this_thread::sleep_for(GRIPPER_SETTLE);
}

static void close_gripper(ArmModule& arm)
{
    std::printf("  gripper: close\n");
    arm.grip();
    std::this_thread::sleep_for(GRIPPER_SETTLE);
}

int main(int argc, char* argv[])
{
    const char* xml_path = "positions.xml";
    std::vector<std::string> trajectory;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a.size() > 4 && a.substr(a.size() - 4) == ".xml")
            xml_path = argv[i];
        else
            trajectory.push_back(a);
    }
    if (trajectory.empty())
        trajectory = DEFAULT_TRAJECTORY;

    auto positions = load_xml(xml_path);
    if (positions.empty()) return 1;

    std::printf("=== Gripper trajectory test ===\n");
    std::printf("Positions file: %s  (%zu entries)\n", xml_path, positions.size());
    std::printf("Trajectory (%zu steps):\n", trajectory.size());
    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& name = trajectory[i];
        bool found = positions.count(name) > 0;
        std::printf("  %zu. %s%s\n", i + 1, name.c_str(), found ? "" : "  NOT FOUND");
        if (!found) {
            std::fprintf(stderr, "ERROR: '%s' not in %s\n", name.c_str(), xml_path);
            return 1;
        }
    }
    std::printf("\nActions:\n");
    std::printf("  after s0_prep: open gripper\n");
    std::printf("  after s0_grasp: close gripper and keep closed\n\n");

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

    bool opened_at_prep = false;
    bool closed_at_grasp = false;

    size_t i = 0;
    size_t chunk_index = 1;
    while (i < trajectory.size()) {
        std::vector<std::string> chunk_names;
        std::vector<std::array<float, 4>> chunk_points;

        const auto current = poll_state(arm).joints;
        chunk_points.push_back(current);

        size_t stop_i = i;
        for (; stop_i < trajectory.size(); ++stop_i) {
            const auto& name = trajectory[stop_i];
            chunk_names.push_back(name);
            chunk_points.push_back(positions.at(name));

            const bool has_gripper_action = (name == "s0_prep" || name == "s0_grasp");
            const bool final_step = (stop_i + 1 == trajectory.size());
            if (has_gripper_action || final_step)
                break;
        }

        const auto& stop_name = trajectory[stop_i];
        const auto& target = positions.at(stop_name);

        std::printf("[chunk %zu] continuous stream -> %s\n", chunk_index, stop_name.c_str());
        print_joints("current:", current);
        for (size_t k = 0; k < chunk_names.size(); ++k)
            std::printf("  waypoint %zu: %s%s\n",
                        k + 1, chunk_names[k].c_str(),
                        (k + 1 == chunk_names.size()) ? "  stop" : "  fly-through");
        print_joints("target:", target);

        float duration = 0.f;
        bool arrived = stream_path(arm, chunk_points, &duration);
        std::printf("  streamed duration: %.2fs\n", duration);
        if (!arrived) {
            auto stuck = poll_state(arm).joints;
            std::fprintf(stderr, "ERROR: timed out at stop '%s'\n", stop_name.c_str());
            print_joints("stuck at:", stuck);
            print_joints("target was:", target);
            print_joint_errors(stuck, target);
            arm.stop();
            return 1;
        }

        print_joints("arrived:", poll_state(arm).joints);

        if (stop_name == "s0_prep") {
            open_gripper(arm);
            opened_at_prep = true;
        } else if (stop_name == "s0_grasp") {
            close_gripper(arm);
            closed_at_grasp = true;
        }

        std::printf("\n");
        i = stop_i + 1;
        ++chunk_index;
    }

    if (!opened_at_prep)
        std::printf("WARNING: trajectory did not include s0_prep, so the gripper was not opened.\n");
    if (!closed_at_grasp)
        std::printf("WARNING: trajectory did not include s0_grasp, so the gripper was not closed.\n");

    std::printf("\nTrajectory complete. No release command was sent after s0_grasp.\n");
    std::printf("Press Enter to power off...\n");
    std::getchar();

    arm.stop();
    return 0;
}
