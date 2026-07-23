#include "states/redred/RedRedState.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace deploy_real
{
namespace
{

struct Quaternion
{
    float w;
    float x;
    float y;
    float z;
};

Quaternion Normalize(Quaternion quaternion)
{
    const float norm = std::sqrt(
        quaternion.w * quaternion.w + quaternion.x * quaternion.x +
        quaternion.y * quaternion.y + quaternion.z * quaternion.z);
    if (!std::isfinite(norm) || norm < 1.0e-6f)
    {
        throw std::runtime_error("RedRed received an invalid quaternion");
    }
    quaternion.w /= norm;
    quaternion.x /= norm;
    quaternion.y /= norm;
    quaternion.z /= norm;
    return quaternion;
}

Quaternion Multiply(const Quaternion &left, const Quaternion &right)
{
    return {
        left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
        left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
        left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
        left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w};
}

Quaternion Conjugate(const Quaternion &quaternion)
{
    return {quaternion.w, -quaternion.x, -quaternion.y, -quaternion.z};
}

Quaternion AxisAngle(float angle, char axis)
{
    const float half_angle = 0.5f * angle;
    const float cosine = std::cos(half_angle);
    const float sine = std::sin(half_angle);
    switch (axis)
    {
    case 'x':
        return {cosine, sine, 0.0f, 0.0f};
    case 'y':
        return {cosine, 0.0f, sine, 0.0f};
    case 'z':
        return {cosine, 0.0f, 0.0f, sine};
    default:
        throw std::runtime_error("unsupported quaternion axis");
    }
}

Quaternion TorsoOrientation(
    const StateContext &context, const RedRedConfig &config)
{
    Quaternion orientation{
        context.robot.base_quaternion[0], context.robot.base_quaternion[1],
        context.robot.base_quaternion[2], context.robot.base_quaternion[3]};
    orientation = Multiply(
        orientation,
        AxisAngle(context.robot.position[config.waist_motor_indices[0]], 'z'));
    orientation = Multiply(
        orientation,
        AxisAngle(context.robot.position[config.waist_motor_indices[1]], 'x'));
    orientation = Multiply(
        orientation,
        AxisAngle(context.robot.position[config.waist_motor_indices[2]], 'y'));
    return Normalize(orientation);
}

float Yaw(const Quaternion &input)
{
    const Quaternion quaternion = Normalize(input);
    return std::atan2(
        2.0f * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0f - 2.0f *
                   (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

std::array<float, 6> RotationColumns(const Quaternion &input)
{
    const Quaternion quaternion = Normalize(input);
    const float ww = quaternion.w * quaternion.w;
    const float xx = quaternion.x * quaternion.x;
    const float yy = quaternion.y * quaternion.y;
    const float zz = quaternion.z * quaternion.z;
    const float wx = quaternion.w * quaternion.x;
    const float wy = quaternion.w * quaternion.y;
    const float wz = quaternion.w * quaternion.z;
    const float xy = quaternion.x * quaternion.y;
    const float xz = quaternion.x * quaternion.z;
    const float yz = quaternion.y * quaternion.z;

    return {
        ww + xx - yy - zz, 2.0f * (xy - wz),
        2.0f * (xy + wz), ww - xx + yy - zz,
        2.0f * (xz - wy), 2.0f * (yz + wx)};
}

std::size_t FindTensorIndex(
    const std::vector<std::string> &names, const std::string &name,
    const char *kind)
{
    const auto tensor = std::find(names.begin(), names.end(), name);
    if (tensor == names.end())
    {
        throw std::runtime_error(
            std::string("RedRed ONNX ") + kind + " is missing: " + name);
    }
    return static_cast<std::size_t>(std::distance(names.begin(), tensor));
}

const Tensor &RequireOutput(
    const TensorMap &outputs, const std::string &name, std::size_t expected_size)
{
    const auto output = outputs.find(name);
    if (output == outputs.end() || output->second.values.size() != expected_size)
    {
        throw std::runtime_error(
            "RedRed returned an unexpected tensor: " + name);
    }
    return output->second;
}

} // namespace

RedRedState::RedRedState(RedRedConfig config)
    : PolicyState(StateId::RedRed, "RedRed", config.model_path),
      config_(std::move(config)),
      previous_action_(config_.num_actions, 0.0f)
{
    ValidateModelContract();
    CacheReferenceOutputs(GenerateReferenceFrame(0));
    ValidateInitialReference();
}

void RedRedState::ValidateModelContract() const
{
    const std::size_t observation_index = FindTensorIndex(
        Runtime().InputNames(), config_.observation_input_name, "input");
    const std::size_t time_step_index = FindTensorIndex(
        Runtime().InputNames(), config_.time_step_input_name, "input");
    const std::size_t action_index = FindTensorIndex(
        Runtime().OutputNames(), config_.action_output_name, "output");
    const std::size_t reference_joint_position_index = FindTensorIndex(
        Runtime().OutputNames(),
        config_.reference_joint_position_output_name, "output");
    const std::size_t reference_joint_velocity_index = FindTensorIndex(
        Runtime().OutputNames(),
        config_.reference_joint_velocity_output_name, "output");
    const std::size_t reference_body_orientation_index = FindTensorIndex(
        Runtime().OutputNames(),
        config_.reference_body_orientation_output_name, "output");

    if (Runtime().InputShape(observation_index) !=
            std::vector<std::int64_t>{1, config_.num_observations} ||
        Runtime().InputShape(time_step_index) != std::vector<std::int64_t>{1, 1} ||
        Runtime().OutputShape(action_index) !=
            std::vector<std::int64_t>{1, config_.num_actions} ||
        Runtime().OutputShape(reference_joint_position_index) !=
            std::vector<std::int64_t>{1, config_.num_actions} ||
        Runtime().OutputShape(reference_joint_velocity_index) !=
            std::vector<std::int64_t>{1, config_.num_actions} ||
        Runtime().OutputShape(reference_body_orientation_index) !=
            std::vector<std::int64_t>{
                1, static_cast<std::int64_t>(config_.tracked_body_count), 4})
    {
        throw std::runtime_error("RedRed ONNX dimensions do not match its YAML config");
    }
}

TensorMap RedRedState::GenerateReferenceFrame(std::size_t frame_index)
{
    if (frame_index >= config_.expected_frames)
    {
        throw std::runtime_error("RedRed reference frame index is out of range");
    }

    TensorMap inputs;
    inputs.emplace(
        config_.observation_input_name,
        Tensor{
            {1, config_.num_observations},
            std::vector<float>(config_.num_observations, 0.0f)});
    inputs.emplace(
        config_.time_step_input_name,
        Tensor{{1, 1}, {static_cast<float>(frame_index)}});
    return Runtime().Run(std::move(inputs));
}

void RedRedState::CacheReferenceOutputs(const TensorMap &outputs)
{
    reference_joint_position_ = RequireOutput(
        outputs, config_.reference_joint_position_output_name,
        kNumMotors).values;
    reference_joint_velocity_ = RequireOutput(
        outputs, config_.reference_joint_velocity_output_name,
        kNumMotors).values;
    reference_body_orientation_ = RequireOutput(
        outputs, config_.reference_body_orientation_output_name,
        config_.tracked_body_count * 4).values;
}

void RedRedState::ValidateInitialReference() const
{
    for (std::size_t policy_idx = 0; policy_idx < kNumMotors; ++policy_idx)
    {
        if (std::abs(
                reference_joint_position_[policy_idx] -
                config_.motion_frame0_joint_angles[policy_idx]) > 1.0e-6f)
        {
            throw std::runtime_error(
                "motion_frame0_joint_angles does not match the embedded ONNX reference");
        }
    }
}

std::size_t RedRedState::NextReferenceFrame() const
{
    if (frame_index_ + 1 < config_.expected_frames)
    {
        return frame_index_ + 1;
    }
    return config_.loop_motion ? 0 : frame_index_;
}

void RedRedState::UpdateProgress(std::size_t completed_frames)
{
    constexpr int kBarWidth = 40;
    const int percent = static_cast<int>(std::min<std::size_t>(
        100, completed_frames * 100 / config_.expected_frames));
    if (percent == last_progress_percent_)
    {
        return;
    }

    const int filled = percent * kBarWidth / 100;
    std::cout << "\r[RedRed] [";
    for (int index = 0; index < kBarWidth; ++index)
    {
        std::cout << (index < filled ? '#' : '-');
    }
    std::cout << "] " << std::setw(3) << percent << "% ("
              << completed_frames << '/' << config_.expected_frames << ')'
              << std::flush;
    last_progress_percent_ = percent;
    progress_line_open_ = true;

    if (completed_frames >= config_.expected_frames)
    {
        FinishProgressLine();
    }
}

void RedRedState::FinishProgressLine()
{
    if (progress_line_open_)
    {
        std::cout << '\n';
        progress_line_open_ = false;
    }
}

void RedRedState::OnEnter(const StateContext &context)
{
    frame_index_ = 0;
    motion_finished_ = false;
    last_progress_percent_ = -1;
    progress_line_open_ = false;
    std::fill(previous_action_.begin(), previous_action_.end(), 0.0f);
    CacheReferenceOutputs(GenerateReferenceFrame(0));

    const Quaternion current_torso_orientation =
        TorsoOrientation(context, config_);
    const std::size_t reference_offset = config_.anchor_body_index * 4;
    const Quaternion initial_reference_orientation = Normalize({
        reference_body_orientation_[reference_offset],
        reference_body_orientation_[reference_offset + 1],
        reference_body_orientation_[reference_offset + 2],
        reference_body_orientation_[reference_offset + 3]});
    reference_yaw_offset_ = std::remainder(
        Yaw(current_torso_orientation) - Yaw(initial_reference_orientation),
        6.28318530717958647692f);
}

TensorMap RedRedState::BuildInputs(const StateContext &context)
{
    std::vector<float> observation;
    observation.reserve(static_cast<std::size_t>(config_.num_observations));

    observation.insert(
        observation.end(),
        reference_joint_position_.begin(), reference_joint_position_.end());
    observation.insert(
        observation.end(),
        reference_joint_velocity_.begin(), reference_joint_velocity_.end());

    const Quaternion torso_orientation = TorsoOrientation(context, config_);

    const std::size_t reference_quaternion_offset =
        config_.anchor_body_index * 4;
    const Quaternion reference_orientation = Normalize({
        reference_body_orientation_[reference_quaternion_offset],
        reference_body_orientation_[reference_quaternion_offset + 1],
        reference_body_orientation_[reference_quaternion_offset + 2],
        reference_body_orientation_[reference_quaternion_offset + 3]});
    const Quaternion aligned_reference_orientation = Multiply(
        AxisAngle(reference_yaw_offset_, 'z'), reference_orientation);
    const std::array<float, 6> anchor_orientation = RotationColumns(
        Multiply(Conjugate(torso_orientation), aligned_reference_orientation));
    observation.insert(
        observation.end(), anchor_orientation.begin(), anchor_orientation.end());
    observation.insert(
        observation.end(), context.robot.angular_velocity.begin(),
        context.robot.angular_velocity.end());

    for (std::size_t policy_idx = 0; policy_idx < kNumMotors; ++policy_idx)
    {
        const std::size_t motor_idx = static_cast<std::size_t>(
            config_.joints.joint_to_motor[policy_idx]);
        observation.push_back(
            context.robot.position[motor_idx] -
            config_.joints.default_angles[policy_idx]);
    }
    for (std::size_t policy_idx = 0; policy_idx < kNumMotors; ++policy_idx)
    {
        const std::size_t motor_idx = static_cast<std::size_t>(
            config_.joints.joint_to_motor[policy_idx]);
        observation.push_back(context.robot.velocity[motor_idx]);
    }
    observation.insert(
        observation.end(), previous_action_.begin(), previous_action_.end());

    if (observation.size() != static_cast<std::size_t>(config_.num_observations))
    {
        throw std::runtime_error("RedRed constructed an invalid observation size");
    }

    TensorMap inputs;
    inputs.emplace(
        config_.observation_input_name,
        Tensor{{1, config_.num_observations}, std::move(observation)});
    inputs.emplace(
        config_.time_step_input_name,
        Tensor{{1, 1}, {static_cast<float>(NextReferenceFrame())}});
    return inputs;
}

void RedRedState::DecodeOutputs(
    const StateContext &, const TensorMap &outputs, MotorCommand &command)
{
    const Tensor &action = RequireOutput(
        outputs, config_.action_output_name,
        static_cast<std::size_t>(config_.num_actions));

    previous_action_ = action.values;
    for (std::size_t policy_idx = 0; policy_idx < kNumMotors; ++policy_idx)
    {
        previous_action_[policy_idx] =
            std::clamp(previous_action_[policy_idx], -100.0f, 100.0f);
        const std::size_t motor_idx = static_cast<std::size_t>(
            config_.joints.joint_to_motor[policy_idx]);
        command.position[motor_idx] =
            previous_action_[policy_idx] * config_.action_scale[policy_idx] +
            config_.joints.default_angles[policy_idx];
        command.kp[motor_idx] = config_.joints.kps[policy_idx];
        command.kd[motor_idx] = config_.joints.kds[policy_idx];
    }

    // The action branch consumes the current observation while time_step
    // prepares the reference cached for the next control cycle.
    CacheReferenceOutputs(outputs);

    UpdateProgress(frame_index_ + 1);

    if (frame_index_ + 1 < config_.expected_frames)
    {
        ++frame_index_;
    }
    else if (config_.loop_motion)
    {
        frame_index_ = 0;
    }
    else
    {
        motion_finished_ = true;
    }
}

StateRequest RedRedState::AfterInference(const StateContext &)
{
    return motion_finished_
               ? StateRequest::Transition(
                     config_.end_state, "RedRed motion finished")
               : StateRequest::None();
}

void RedRedState::OnExit(const StateContext &)
{
    FinishProgressLine();
    frame_index_ = 0;
    reference_yaw_offset_ = 0.0f;
    motion_finished_ = false;
    reference_joint_position_.clear();
    reference_joint_velocity_.clear();
    reference_body_orientation_.clear();
    last_progress_percent_ = -1;
    std::fill(previous_action_.begin(), previous_action_.end(), 0.0f);
}

} // namespace deploy_real
