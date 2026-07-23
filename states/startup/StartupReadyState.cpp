#include "states/startup/StartupReadyState.h"

#include <algorithm>
#include <chrono>
#include <utility>

namespace deploy_real
{
namespace
{

constexpr float kStartupDurationSeconds = 2.0f;

} // namespace

StartupReadyState::StartupReadyState(JointProfile profile)
    : profile_(std::move(profile))
{
}

StateId StartupReadyState::Id() const
{
    return StateId::StartupReady;
}

const char *StartupReadyState::Name() const
{
    return ToString(Id());
}

void StartupReadyState::Enter(const StateContext &context)
{
    initial_position_ = context.robot.position;
    start_time_ = context.now;
    ready_ = false;
}

StateRequest StartupReadyState::Update(
    const StateContext &context, MotorCommand &command)
{
    command = {};
    const float elapsed =
        std::chrono::duration<float>(context.now - start_time_).count();
    const float phase = std::clamp(elapsed / kStartupDurationSeconds, 0.0f, 1.0f);

    for (std::size_t policy_idx = 0; policy_idx < kNumMotors; ++policy_idx)
    {
        const std::size_t motor_idx =
            static_cast<std::size_t>(profile_.joint_to_motor[policy_idx]);
        command.position[motor_idx] =
            (1.0f - phase) * initial_position_[motor_idx] +
            phase * profile_.default_angles[policy_idx];
        command.kp[motor_idx] = profile_.kps[policy_idx];
        command.kd[motor_idx] = profile_.kds[policy_idx];
    }
    ready_ = phase >= 1.0f;
    return StateRequest::None();
}

void StartupReadyState::Exit(const StateContext &) {}

bool StartupReadyState::CanTransitionTo(StateId next) const
{
    return (next != StateId::Loco && next != StateId::RedRed) || ready_;
}

} // namespace deploy_real
