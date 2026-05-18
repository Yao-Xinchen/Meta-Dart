#include "trigger.hpp"
#include "config.hpp"
#include "log.hpp"
#include "mg995_servo.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace {

constexpr auto TRIGGER_LOOP_PERIOD = 20ms;

std::chrono::milliseconds seconds_to_ms(float seconds)
{
    if (seconds < 0.f)
        seconds = 0.f;
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<float>(seconds));
}

}  // namespace

TriggerModule::TriggerModule() = default;

TriggerModule::~TriggerModule()
{
    stop();
}

bool TriggerModule::start()
{
    const auto chip = mg995::resolve_pwmchip(TRIGGER_PWM_CHIP);
    if (chip.empty()) {
        mdlog::error("Trigger",
                     "cannot find MG995 PWM chip '%s'; enable pwm0-m1 or set TRIGGER_PWM_CHIP",
                     TRIGGER_PWM_CHIP);
        hw_ok_ = false;
        publish_state();
        return false;
    }

    pwm_ = std::make_unique<mg995::SysfsPwm>(chip, TRIGGER_PWM_CHANNEL);
    if (!pwm_->open()) {
        mdlog::error("Trigger", "failed to open MG995 PWM %s pwm%d",
                     chip.c_str(), TRIGGER_PWM_CHANNEL);
        pwm_.reset();
        hw_ok_ = false;
        publish_state();
        return false;
    }

    hw_ok_ = true;
    commanded_position_ = TriggerState::Position::HoldingHigh;
    command_time_ = std::chrono::steady_clock::now();
    last_position_deg_ = TRIGGER_HOLD_POS_DEG;
    if (!write_trigger_position(TRIGGER_HOLD_POS_DEG)) {
        hw_ok_ = false;
        publish_state();
        return false;
    }

    running_ = true;
    thread_ = std::thread(&TriggerModule::loop, this);
    mdlog::ok("Trigger", "started command-only trigger interface");
    return true;
}

void TriggerModule::stop()
{
    running_ = false;
    if (thread_.joinable())
        thread_.join();

    if (hw_ok_)
        write_trigger_position(TRIGGER_HOLD_POS_DEG);
    if (pwm_)
        pwm_->disable();
}

void TriggerModule::hold()
{
    TriggerCmd cmd;
    cmd.type = TriggerCmd::Type::Hold;
    cmd_buf_.write(cmd);
}

void TriggerModule::release()
{
    TriggerCmd cmd;
    cmd.type = TriggerCmd::Type::Release;
    cmd_buf_.write(cmd);
}

bool TriggerModule::read_state(TriggerState& out)
{
    return state_buf_.read(out);
}

void TriggerModule::loop()
{
    TriggerCmd cmd;

    while (running_) {
        if (cmd_buf_.read(cmd))
            execute_cmd(cmd);
        publish_state();
        std::this_thread::sleep_for(TRIGGER_LOOP_PERIOD);
    }
}

void TriggerModule::execute_cmd(const TriggerCmd& cmd)
{
    command_time_ = std::chrono::steady_clock::now();

    switch (cmd.type) {
    case TriggerCmd::Type::Hold:
        commanded_position_ = TriggerState::Position::HoldingHigh;
        if (!write_trigger_position(TRIGGER_HOLD_POS_DEG))
            hw_ok_ = false;
        mdlog::event("Trigger", "command high/hold");
        break;

    case TriggerCmd::Type::Release:
        commanded_position_ = TriggerState::Position::ReleasingLow;
        if (!write_trigger_position(TRIGGER_RELEASE_POS_DEG))
            hw_ok_ = false;
        mdlog::event("Trigger", "command low/release");
        break;
    }
}

bool TriggerModule::write_trigger_position(float position_deg)
{
    if (!pwm_)
        return false;

    const bool ok = pwm_->move_slow(last_position_deg_,
                                    position_deg,
                                    mg995::DEFAULT_STEP_DEG,
                                    mg995::DEFAULT_STEP_DELAY);
    if (ok)
        last_position_deg_ = mg995::clamp_angle(position_deg);
    return ok;
}

void TriggerModule::publish_state()
{
    TriggerState state{};
    state.position = commanded_position_;
    state.hw_ok = hw_ok_;
    state.inferred = true;

    const auto settle = seconds_to_ms(TRIGGER_SETTLE_TIME_S);
    state.settled = (std::chrono::steady_clock::now() - command_time_) >= settle;

    state_buf_.write(state);
}
