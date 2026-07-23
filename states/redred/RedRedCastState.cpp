#include "states/redred/RedRedCastState.h"

#include <algorithm>
#include <utility>

namespace deploy_real
{

RedRedCastState::RedRedCastState(
    LocoConfig loco_config, RedRedConfig redred_config)
    : LocoState(
          StateId::RedRedCast, "RedRedCast", std::move(loco_config)),
      redred_config_(std::move(redred_config))
{
    for (std::size_t policy_idx = 0; policy_idx < kNumMotors; ++policy_idx)
    {
        const std::size_t motor_idx = static_cast<std::size_t>(
            redred_config_.joints.joint_to_motor[policy_idx]);
        motor_to_policy_[motor_idx] = policy_idx;
    }
}

void RedRedCastState::OnEnter(const StateContext &context)
{
    LocoState::OnEnter(context);
    initial_position_ = context.robot.position;
    start_time_ = context.now;
}

std::array<float, 3> RedRedCastState::CommandVelocity(
    const StateContext &)
{
    return {};
}

float RedRedCastState::InterpolationPhase(const StateContext &context) const
{
    const float elapsed =
        std::chrono::duration<float>(context.now - start_time_).count();
    return std::clamp(elapsed / redred_config_.cast_duration, 0.0f, 1.0f);
}

void RedRedCastState::DecodeOutputs(
    const StateContext &context, const TensorMap &outputs,
    MotorCommand &command)
{
    LocoState::DecodeOutputs(context, outputs, command);

    const float phase = InterpolationPhase(context);
    for (const std::size_t motor_idx : redred_config_.upper_body_motor_indices)
    {
        const std::size_t policy_idx = motor_to_policy_[motor_idx];
        const float target_position =
            redred_config_.motion_frame0_joint_angles[policy_idx];
        command.position[motor_idx] =
            (1.0f - phase) * initial_position_[motor_idx] +
            phase * target_position;
        command.kp[motor_idx] = redred_config_.joints.kps[policy_idx];
        command.kd[motor_idx] = redred_config_.joints.kds[policy_idx];
    }
}

StateRequest RedRedCastState::AfterInference(const StateContext &context)
{
    return InterpolationPhase(context) >= 1.0f
               ? StateRequest::Transition(
                     StateId::RedRed, "RedRed upper-body cast complete")
               : StateRequest::None();
}

} // namespace deploy_real
