#include "decision.hpp"
#include "config.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────────────────────
// Named poses — fill these in once the physical setup is calibrated.
//
// HOME: safe resting position with the arm upright.
// PLACE: position above the track loading point where the dart is released.
// ─────────────────────────────────────────────────────────────────────────────
static const ArmCmd HOME_CMD = {
    .type   = ArmCmd::Type::MoveJoint,
    .joints = {0.f, -1.05f, 0.35f, 0.70f},  // TODO: tune on hardware
};

static const ArmCmd PLACE_CMD = {
    .type  = ArmCmd::Type::MovePose,
    .px    = 0.20f,  // TODO: measure actual place position
    .py    = 0.00f,
    .pz    = 0.05f,
    .pitch = -1.57f,  // pointing down
};

// ─────────────────────────────────────────────────────────────────────────────

static ArmCmd make_move_joint_cmd(const std::array<float, 4>& joints)
{
    ArmCmd cmd;
    cmd.type = ArmCmd::Type::MoveJoint;
    cmd.joints = joints;
    return cmd;
}

static ArmCmd make_grip_cmd()
{
    ArmCmd cmd;
    cmd.type = ArmCmd::Type::Grip;
    return cmd;
}

static ArmCmd make_release_cmd()
{
    ArmCmd cmd;
    cmd.type = ArmCmd::Type::Release;
    return cmd;
}

static bool in_region(float x, float y, const PickSlotConfig& s)
{
    return s.enabled &&
           x >= s.x_min && x <= s.x_max &&
           y >= s.y_min && y <= s.y_max;
}

// ─────────────────────────────────────────────────────────────────────────────

DecisionModule::DecisionModule(TripleBuffer<Detection>& detection_buf,
                               TripleBuffer<ArmState>&  arm_state_buf,
                               TripleBuffer<ArmCmd>&    arm_cmd_buf)
    : detection_buf_(detection_buf)
    , arm_state_buf_(arm_state_buf)
    , arm_cmd_buf_(arm_cmd_buf)
{}

DecisionModule::~DecisionModule() { stop(); }

bool DecisionModule::start() {
    running_ = true;
    state_   = State::Idle;
    thread_  = std::thread(&DecisionModule::loop, this);
    return true;
}

void DecisionModule::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void DecisionModule::send_cmd(const ArmCmd& cmd) {
    arm_cmd_buf_.write(cmd);
}

DecisionModule::PickSlot DecisionModule::classify_detection(const Detection& det) const {
    if (!det.valid) return PickSlot::None;
    for (int i = 0; i < static_cast<int>(PICK_SLOTS.size()); ++i) {
        if (in_region(det.x, det.y, PICK_SLOTS[i]))
            return static_cast<PickSlot>(i + 1);  // None=0, Position1=1, ...
    }
    return PickSlot::None;
}

bool DecisionModule::slot_is_calibrated(PickSlot slot) const {
    if (slot == PickSlot::None) return false;
    return PICK_SLOTS[static_cast<int>(slot) - 1].calibrated;
}

const std::array<float, 4>& DecisionModule::slot_hover_joints(PickSlot slot) const {
    if (slot == PickSlot::None) {
        std::fprintf(stderr, "[Decision] BUG: slot_hover_joints called with None\n");
        return PICK_SLOTS[0].hover_joints;
    }
    return PICK_SLOTS[static_cast<int>(slot) - 1].hover_joints;
}

const std::array<float, 4>& DecisionModule::slot_pick_joints(PickSlot slot) const {
    if (slot == PickSlot::None) {
        std::fprintf(stderr, "[Decision] BUG: slot_pick_joints called with None\n");
        return PICK_SLOTS[0].pick_joints;
    }
    return PICK_SLOTS[static_cast<int>(slot) - 1].pick_joints;
}

void DecisionModule::report_detection_region(const Detection& det, PickSlot slot) {
    if (!det.valid) {
        if (last_reported_detection_valid_) {
            std::printf("[Decision] No valid detection\n");
            last_reported_detection_valid_ = false;
            last_reported_slot_ = PickSlot::None;
        }
        return;
    }

    if (last_reported_detection_valid_ && slot == last_reported_slot_) return;

    if (slot == PickSlot::None) {
        std::printf("[Decision] Detection at (%.3f, %.3f)m is outside configured regions\n",
                    det.x, det.y);
    } else {
        std::printf("[Decision] Detection at (%.3f, %.3f)m is in %s\n",
                    det.x, det.y, pick_slot_name(slot));
    }

    last_reported_detection_valid_ = true;
    last_reported_slot_ = slot;
}

void DecisionModule::transition_to(State next) {
    if (next == State::Searching)
        last_warned_uncalibrated_slot_ = PickSlot::None;
    state_ = next;
    state_entered_ = std::chrono::steady_clock::now();
}

void DecisionModule::start_pick(PickSlot slot) {
    active_slot_ = slot;

    switch (slot) {
    case PickSlot::Position1:
    case PickSlot::Position2:
    case PickSlot::Position3:
    case PickSlot::Position4:
        std::printf("[Decision] %s matched; moving to hover joints\n", pick_slot_name(slot));
        send_cmd(make_move_joint_cmd(slot_hover_joints(slot)));
        transition_to(State::MovingToPickHover);
        break;
    case PickSlot::None:
    default:
        break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// State machine
// ─────────────────────────────────────────────────────────────────────────────
void DecisionModule::loop() {
    constexpr auto period = std::chrono::microseconds(1'000'000 / DECISION_LOOP_HZ);
    using Clock = std::chrono::steady_clock;

    Detection  det;
    ArmState   arm;
    State      prev_state = State::Idle;

    if (ALLOW_REGION_REPORTING_WITHOUT_ARM_HW) {
        transition_to(State::Searching);
    } else {
        send_cmd(HOME_CMD);
        transition_to(State::MovingHome);
    }

    while (running_) {
        auto t0 = Clock::now();

        // Read latest data (non-blocking)
        detection_buf_.read(det);
        arm_state_buf_.read(arm);

        if (state_ != prev_state) {
            std::printf("[Decision] → %s\n", state_name(state_));
            prev_state = state_;
        }

        switch (state_) {

        case State::Idle:
            // Nothing to do — wait for external trigger or restart
            break;

        case State::MovingHome:
            if (arm.reached_goal) {
                send_cmd(make_release_cmd());
                transition_to(State::Releasing);
            }
            break;

        case State::Searching:
            {
                PickSlot slot = classify_detection(det);
                report_detection_region(det, slot);
                if (!det.valid || slot == PickSlot::None) break;

                if (ALLOW_REGION_REPORTING_WITHOUT_ARM_HW) break;
                if (!arm.hw_ok) break;

                if (!slot_is_calibrated(slot)) {
                    if (slot != last_warned_uncalibrated_slot_) {
                        std::printf("[Decision] %s detected at (%.3f, %.3f)m, "
                                    "but its joint table is not calibrated yet; ignoring\n",
                                    pick_slot_name(slot), det.x, det.y);
                        last_warned_uncalibrated_slot_ = slot;
                    }
                    break;
                }

                std::printf("[Decision] Detection (%.3f, %.3f, %.3f)m classified for pick\n",
                            det.x, det.y, det.z);
                start_pick(slot);
            }
            break;

        case State::MovingToPickHover:
            if (arm.reached_goal) {
                std::printf("[Decision] Hover reached; FK EE=(%.3f, %.3f, %.3f)m\n",
                            arm.px, arm.py, arm.pz);
                send_cmd(make_move_joint_cmd(slot_pick_joints(active_slot_)));
                transition_to(State::MovingToPick);
            }
            break;

        case State::MovingToPick:
            if (arm.reached_goal) {
                std::printf("[Decision] Pick reached; FK EE=(%.3f, %.3f, %.3f)m\n",
                            arm.px, arm.py, arm.pz);
                send_cmd(make_grip_cmd());
                transition_to(State::Gripping);
            }
            break;

        case State::Gripping:
            if (Clock::now() - state_entered_ >=
                std::chrono::milliseconds(GRIPPER_CLOSE_SETTLE_MS)) {
                send_cmd(make_move_joint_cmd(slot_hover_joints(active_slot_)));
                transition_to(State::Lifting);
            }
            break;

        case State::Lifting:
            if (arm.reached_goal) {
                send_cmd(PLACE_CMD);
                transition_to(State::MovingToPlace);
            }
            break;

        case State::MovingToPlace:
            if (arm.reached_goal) {
                send_cmd(make_release_cmd());
                transition_to(State::Releasing);
            }
            break;

        case State::Releasing:
            if (Clock::now() - state_entered_ >=
                std::chrono::milliseconds(GRIPPER_OPEN_SETTLE_MS)) {
                if (active_slot_ == PickSlot::None) {
                    transition_to(State::Searching);
                } else {
                    active_slot_ = PickSlot::None;
                    send_cmd(HOME_CMD);
                    transition_to(State::MovingHome);
                }
            }
            break;
        }

        std::this_thread::sleep_until(t0 + period);
    }
}

const char* DecisionModule::pick_slot_name(PickSlot slot) const {
    switch (slot) {
    case PickSlot::Position1: return "Region 1";
    case PickSlot::Position2: return "Region 2";
    case PickSlot::Position3: return "Region 3";
    case PickSlot::Position4: return "Region 4";
    case PickSlot::None:
    default:                  return "No region";
    }
}

const char* DecisionModule::state_name(State s) const {
    switch (s) {
    case State::Idle:                return "IDLE";
    case State::MovingHome:          return "MOVING_HOME";
    case State::Searching:           return "SEARCHING";
    case State::MovingToPickHover:   return "MOVING_TO_PICK_HOVER";
    case State::MovingToPick:        return "MOVING_TO_PICK";
    case State::Gripping:            return "GRIPPING";
    case State::Lifting:             return "LIFTING";
    case State::MovingToPlace:       return "MOVING_TO_PLACE";
    case State::Releasing:           return "RELEASING";
    default:                         return "?";
    }
}
