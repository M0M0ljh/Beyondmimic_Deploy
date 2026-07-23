#include "states/loco/LocoState.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>

namespace deploy_real
{
namespace
{

constexpr std::size_t kBaseObservationSize = 9;

} // namespace

LocoState::LocoState(LocoConfig config, float entry_blend_duration)
    : LocoState(
          StateId::Loco, "Loco", std::move(config), entry_blend_duration)
{
}

LocoState::LocoState(
    StateId id, std::string name, LocoConfig config,
    float entry_blend_duration)
    : PolicyState(id, std::move(name), config.model_path),
      config_(std::move(config)),
      previous_action_(config_.num_actions, 0.0f),
      entry_blend_duration_(entry_blend_duration)
{
    if (!std::isfinite(entry_blend_duration_) || entry_blend_duration_ < 0.0f)
    {
        throw std::runtime_error("Loco entry blend duration must be non-negative");
    }
    ValidateModelContract();
}

void LocoState::ValidateModelContract() const
{
    if (Runtime().InputNames().size() != 1 || Runtime().OutputNames().size() != 1)
    {
        throw std::runtime_error("Loco ONNX model must have one input and one output");
    }
    const auto &input_shape = Runtime().InputShape(0);
    const auto &output_shape = Runtime().OutputShape(0);
    if (input_shape.empty() || input_shape.back() != config_.num_observations ||
        output_shape.empty() || output_shape.back() != config_.num_actions)
    {
        throw std::runtime_error("Loco ONNX dimensions do not match its YAML config");
    }
}

void LocoState::OnEnter(const StateContext &context)
{
    std::fill(previous_action_.begin(), previous_action_.end(), 0.0f);
    previous_planar_command_.fill(0.0f);
    entry_position_ = context.robot.position;
    entry_start_time_ = context.now;
}

float LocoState::EntryBlendPhase(const StateContext &context) const
{
    if (entry_blend_duration_ <= 0.0f)
    {
        return 1.0f;
    }
    const float elapsed =
        std::chrono::duration<float>(context.now - entry_start_time_).count();
    return std::clamp(elapsed / entry_blend_duration_, 0.0f, 1.0f);
}

TensorMap LocoState::BuildInputs(const StateContext &context)
{
    std::vector<float> observation(config_.num_observations, 0.0f);
    const std::array<float, 3> command_velocity = CommandVelocity(context);
    for (std::size_t i = 0; i < 3; ++i)
    {
        observation[i] =
            context.robot.angular_velocity[i] * config_.angular_velocity_scale;
        observation[i + 3] = context.robot.gravity[i];
        observation[i + 6] =
            command_velocity[i] * config_.maximum_command[i] *
            config_.command_scale[i];
    }

    const std::size_t position_offset = kBaseObservationSize;
    const std::size_t velocity_offset = position_offset + kNumMotors;
    const std::size_t action_offset = velocity_offset + kNumMotors;
    for (std::size_t policy_idx = 0; policy_idx < kNumMotors; ++policy_idx)
    {
        const std::size_t motor_idx = static_cast<std::size_t>(
            config_.joints.joint_to_motor[policy_idx]);
        observation[position_offset + policy_idx] =
            (context.robot.position[motor_idx] -
             config_.joints.default_angles[policy_idx]) *
            config_.joint_position_scale;
        observation[velocity_offset + policy_idx] =
            context.robot.velocity[motor_idx] * config_.joint_velocity_scale;
        observation[action_offset + policy_idx] = previous_action_[policy_idx];
    }
    for (float &value : observation)
    {
        value = std::clamp(value, -100.0f, 100.0f);
    }

    TensorMap inputs;
    inputs.emplace(
        Runtime().InputNames().front(),
        Tensor{{1, config_.num_observations}, std::move(observation)});
    return inputs;
}

std::array<float, 3> LocoState::CommandVelocity(
    const StateContext &context)
{
    std::array<float, 3> command = context.input.velocity_command;
    for (std::size_t index = 0; index < previous_planar_command_.size(); ++index)
    {
        if (std::abs(command[index]) <= config_.planar_command_dead_zone)
        {
            command[index] = 0.0f;
        }
        previous_planar_command_[index] =
            config_.planar_command_smoothing * previous_planar_command_[index] +
            (1.0f - config_.planar_command_smoothing) * command[index];
        command[index] = previous_planar_command_[index];
    }
    return command;
}

void LocoState::DecodeOutputs(
    const StateContext &context, const TensorMap &outputs, MotorCommand &command)
{
    const auto output = outputs.find(Runtime().OutputNames().front());
    if (output == outputs.end() ||
        output->second.values.size() != static_cast<std::size_t>(config_.num_actions))
    {
        throw std::runtime_error("Loco returned an unexpected action tensor");
    }

    previous_action_ = output->second.values;
    for (std::size_t policy_idx = 0; policy_idx < kNumMotors; ++policy_idx)
    {
        previous_action_[policy_idx] =
            std::clamp(previous_action_[policy_idx], -100.0f, 100.0f);
        const std::size_t motor_idx = static_cast<std::size_t>(
            config_.joints.joint_to_motor[policy_idx]);
        command.position[motor_idx] =
            previous_action_[policy_idx] * config_.action_scale +
            config_.joints.default_angles[policy_idx];
        command.kp[motor_idx] = config_.joints.kps[policy_idx];
        command.kd[motor_idx] = config_.joints.kds[policy_idx];
    }

    const float blend_phase = EntryBlendPhase(context);
    for (std::size_t motor_idx = 0; motor_idx < kNumMotors; ++motor_idx)
    {
        command.position[motor_idx] =
            (1.0f - blend_phase) * entry_position_[motor_idx] +
            blend_phase * command.position[motor_idx];
    }
}

} // namespace deploy_real
