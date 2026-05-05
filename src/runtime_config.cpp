#include "runtime_config.hpp"

#include <opencv2/core.hpp>

#include <algorithm>
#include <cctype>
#include <cstdio>

namespace {

std::string lower_copy(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

bool parse_gripper_action(const std::string& text, GripperAction& out)
{
    const std::string value = lower_copy(text);
    if (value.empty() || value == "none") {
        out = GripperAction::None;
        return true;
    }
    if (value == "open" || value == "release") {
        out = GripperAction::Open;
        return true;
    }
    if (value == "close" || value == "grip") {
        out = GripperAction::Close;
        return true;
    }
    return false;
}

void read_float(const cv::FileNode& parent, const char* key, float& out)
{
    const cv::FileNode node = parent[key];
    if (!node.empty() && (node.isReal() || node.isInt()))
        out = static_cast<float>((double)node);
}

void read_string(const cv::FileNode& parent, const char* key, std::string& out)
{
    const cv::FileNode node = parent[key];
    if (!node.empty() && node.isString())
        out = static_cast<std::string>(node);
}

void read_string_sequence(const cv::FileNode& parent,
                          const char* key,
                          std::vector<std::string>& out)
{
    const cv::FileNode node = parent[key];
    if (node.empty() || !node.isSeq())
        return;

    std::vector<std::string> values;
    for (const auto& item : node) {
        if (item.isString())
            values.push_back(static_cast<std::string>(item));
    }
    if (!values.empty())
        out = values;
}

void read_zones(const cv::FileNode& parent, DecisionRuntimeConfig& decision)
{
    const cv::FileNode node = parent["zones"];
    if (node.empty() || !node.isSeq())
        return;

    std::vector<DecisionZone> zones;
    for (const auto& item : node) {
        if (!item.isMap())
            continue;

        DecisionZone zone;
        read_string(item, "name", zone.name);
        read_float(item, "center_x", zone.cx);
        read_float(item, "center_y", zone.cy);
        read_float(item, "half_width", zone.half_w);
        read_float(item, "half_height", zone.half_h);
        read_string(item, "trajectory", zone.trajectory);

        if (!zone.name.empty() && !zone.trajectory.empty())
            zones.push_back(zone);
    }

    if (!zones.empty())
        decision.zones = zones;
}

void read_trajectories(const cv::FileNode& parent,
                       DecisionRuntimeConfig& decision,
                       std::string& error)
{
    const cv::FileNode node = parent["trajectories"];
    if (node.empty() || !node.isMap())
        return;

    std::map<std::string, std::vector<DecisionTrajectoryStep>> trajectories;
    for (auto it = node.begin(); it != node.end(); ++it) {
        const cv::FileNode trajectory_node = *it;
        if (!trajectory_node.isSeq())
            continue;

        const std::string name = trajectory_node.name();
        std::vector<DecisionTrajectoryStep> steps;

        for (const auto& step_node : trajectory_node) {
            if (!step_node.isMap())
                continue;

            DecisionTrajectoryStep step;
            read_string(step_node, "position", step.position);

            std::string action_text;
            read_string(step_node, "gripper", action_text);
            if (!parse_gripper_action(action_text, step.gripper)) {
                error = "invalid gripper action '" + action_text +
                        "' in trajectory '" + name + "'";
                return;
            }

            if (!step.position.empty())
                steps.push_back(step);
        }

        if (!name.empty() && !steps.empty())
            trajectories[name] = steps;
    }

    if (!trajectories.empty())
        decision.trajectories = trajectories;
}

}  // namespace

const char* gripper_action_name(GripperAction action)
{
    switch (action) {
    case GripperAction::None:  return "none";
    case GripperAction::Open:  return "open";
    case GripperAction::Close: return "close";
    default:                   return "?";
    }
}

RuntimeConfig make_default_runtime_config()
{
    RuntimeConfig config;

    config.decision.zones = {
        {"slot_1", -0.062f,  0.029f, 0.020f, 0.020f, "slot_1"},
        {"slot_2",  0.068f,  0.026f, 0.020f, 0.020f, "slot_2"},
        {"slot_3",  0.105f, -0.008f, 0.020f, 0.020f, "slot_3"},
    };

    const auto common_tail = std::vector<DecisionTrajectoryStep>{
        {"backward_prep", GripperAction::None},
        {"forward_prep",  GripperAction::None},
        {"loading",       GripperAction::Open},
        {"home",          GripperAction::None},
    };

    auto make_slot = [&](const char* prefix) {
        std::vector<DecisionTrajectoryStep> steps = {
            {std::string(prefix) + "_prep",       GripperAction::Open},
            {std::string(prefix) + "_grasp",      GripperAction::Close},
            {std::string(prefix) + "_pulled_out", GripperAction::None},
        };
        steps.insert(steps.end(), common_tail.begin(), common_tail.end());
        return steps;
    };

    config.decision.trajectories["slot_0"] = make_slot("s0");
    config.decision.trajectories["slot_1"] = make_slot("s1");
    config.decision.trajectories["slot_2"] = make_slot("s2");
    config.decision.trajectories["slot_3"] = make_slot("s3");

    return config;
}

bool load_runtime_config_json(const std::string& path,
                              RuntimeConfig& config,
                              std::string& error)
{
    cv::FileStorage fs;
    try {
        fs.open(path, cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
    } catch (const cv::Exception& e) {
        error = e.what();
        return false;
    }

    if (!fs.isOpened()) {
        error = "cannot open " + path;
        return false;
    }

    const cv::FileNode vision = fs["vision"];
    if (!vision.empty() && vision.isMap())
        read_float(vision, "confidence_threshold", config.vision_confidence_threshold);

    const cv::FileNode decision = fs["decision"];
    if (!decision.empty() && decision.isMap()) {
        read_float(decision, "stable_dart_time_s", config.decision.stable_dart_time_s);
        read_float(decision, "homing_move_duration_s", config.decision.homing_move_duration_s);
        read_float(decision, "min_segment_duration_s", config.decision.min_segment_duration_s);
        read_float(decision, "max_segment_duration_s", config.decision.max_segment_duration_s);
        read_float(decision, "joint_cruise_speed_rad_s", config.decision.joint_cruise_speed_rad_s);
        read_float(decision, "flythrough_joint_threshold_rad", config.decision.flythrough_joint_threshold_rad);
        read_float(decision, "move_timeout_s", config.decision.move_timeout_s);
        read_float(decision, "gripper_settle_s", config.decision.gripper_settle_s);
        read_float(decision, "waypoint_settle_ms", config.decision.waypoint_settle_ms);
        read_float(decision, "reset_cooldown_s", config.decision.reset_cooldown_s);
        read_string(decision, "search_wait_pose", config.decision.search_wait_pose);
        read_string_sequence(decision, "search_ready_sequence", config.decision.search_ready_sequence);
        read_zones(decision, config.decision);
        read_trajectories(decision, config.decision, error);
        if (!error.empty())
            return false;
    }

    return true;
}
