#pragma once

#include "config.hpp"

#include <map>
#include <string>
#include <vector>

enum class GripperAction {
    None,
    Open,
    Close,
};

struct DecisionTrajectoryStep {
    std::string   position;
    GripperAction gripper = GripperAction::None;
};

struct DecisionZone {
    std::string name;
    float       cx = 0.f;
    float       cy = 0.f;
    float       half_w = 0.020f;
    float       half_h = 0.020f;
    std::string trajectory;
};

struct DecisionRuntimeConfig {
    float stable_dart_time_s = 2.0f;
    float homing_move_duration_s = 4.0f;
    float min_segment_duration_s = 2.0f;
    float max_segment_duration_s = 9.0f;
    float joint_cruise_speed_rad_s = 0.45f;
    float flythrough_joint_threshold_rad = 0.12f;
    float move_timeout_s = 15.0f;
    float gripper_settle_s = 1.0f;
    float waypoint_settle_ms = 300.0f;
    float reset_cooldown_s = 2.0f;

    std::string search_wait_pose = "backward_prep";
    std::vector<std::string> search_ready_sequence = {
        "home",
        "forward_prep",
        "backward_prep",
    };

    std::vector<DecisionZone> zones;
    std::map<std::string, std::vector<DecisionTrajectoryStep>> trajectories;
};

struct RuntimeConfig {
    float vision_confidence_threshold = CONF_THRESHOLD;
    DecisionRuntimeConfig decision;
};

RuntimeConfig make_default_runtime_config();
bool load_runtime_config_json(const std::string& path,
                              RuntimeConfig& config,
                              std::string& error);
const char* gripper_action_name(GripperAction action);
