#include "states/redred/RedRedConfig.h"

#include "common/ProjectPath.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace deploy_real
{
namespace
{

template <typename T, std::size_t Size>
std::array<T, Size> LoadArray(const YAML::Node &yaml, const char *name)
{
    const std::vector<T> values = yaml[name].as<std::vector<T>>();
    if (values.size() != Size)
    {
        throw std::runtime_error(
            std::string(name) + " must contain " + std::to_string(Size) +
            " values, got " + std::to_string(values.size()));
    }
    std::array<T, Size> result{};
    std::copy(values.begin(), values.end(), result.begin());
    return result;
}

void ValidateJointMapping(const std::array<int, kNumMotors> &mapping)
{
    std::array<bool, kNumMotors> seen{};
    for (const int motor_idx : mapping)
    {
        if (motor_idx < 0 || motor_idx >= static_cast<int>(kNumMotors) || seen[motor_idx])
        {
            throw std::runtime_error(
                "joint2motor_idx must be a permutation of motor indices 0..28");
        }
        seen[motor_idx] = true;
    }
}

void ValidateUpperBodyMotorIndices(const std::vector<std::size_t> &indices)
{
    constexpr std::size_t kFirstUpperBodyMotor = 15;
    constexpr std::size_t kUpperBodyMotorCount =
        kNumMotors - kFirstUpperBodyMotor;
    if (indices.size() != kUpperBodyMotorCount)
    {
        throw std::runtime_error(
            "upper_body_motor_idx must contain motor indices 15..28");
    }

    std::array<bool, kNumMotors> seen{};
    for (const std::size_t motor_idx : indices)
    {
        if (motor_idx < kFirstUpperBodyMotor || motor_idx >= kNumMotors ||
            seen[motor_idx])
        {
            throw std::runtime_error(
                "upper_body_motor_idx must be a permutation of motor indices 15..28");
        }
        seen[motor_idx] = true;
    }
}

StateId ParseEndState(const std::string &name)
{
    if (name == "loco")
    {
        return StateId::Loco;
    }
    if (name == "startup_ready")
    {
        return StateId::StartupReady;
    }
    throw std::runtime_error(
        "motion.end_state must be 'loco' or 'startup_ready'");
}

std::size_t FindJointMotorIndex(
    const std::vector<std::string> &joint_names,
    const std::array<int, kNumMotors> &mapping,
    const std::string &joint_name)
{
    const auto joint = std::find(joint_names.begin(), joint_names.end(), joint_name);
    if (joint == joint_names.end())
    {
        throw std::runtime_error("RedRed joint is missing: " + joint_name);
    }
    return static_cast<std::size_t>(mapping[std::distance(joint_names.begin(), joint)]);
}

} // namespace

RedRedConfig RedRedConfig::Load(const std::string &config_path)
{
    const YAML::Node yaml = YAML::LoadFile(config_path);
    RedRedConfig config;
    config.control_dt = yaml["control_dt"].as<float>();
    config.model_path = ProjectPath(yaml["policy_path"].as<std::string>());
    config.num_observations = yaml["num_observations"].as<int>();
    config.num_actions = yaml["num_actions"].as<int>();
    config.observation_input_name =
        yaml["model_inputs"]["observation"]["name"].as<std::string>();
    config.time_step_input_name =
        yaml["model_inputs"]["time_step"]["name"].as<std::string>();
    config.action_output_name =
        yaml["model_outputs"]["actions"]["name"].as<std::string>();
    config.reference_joint_position_output_name =
        yaml["model_outputs"]["reference_joint_position"]["name"].as<std::string>();
    config.reference_joint_velocity_output_name =
        yaml["model_outputs"]["reference_joint_velocity"]["name"].as<std::string>();
    config.reference_body_orientation_output_name =
        yaml["model_outputs"]["reference_body_orientation"]["name"].as<std::string>();
    config.loop_motion = yaml["motion"]["loop"].as<bool>();
    config.end_state =
        ParseEndState(yaml["motion"]["end_state"].as<std::string>());
    config.cast_duration = yaml["cast_duration"].as<float>();
    config.return_duration = yaml["return_duration"].as<float>();
    config.motion_fps = yaml["motion"]["fps"].as<float>();
    config.upper_body_motor_indices =
        yaml["upper_body_motor_idx"].as<std::vector<std::size_t>>();
    config.expected_frames = yaml["motion"]["expected_frames"].as<std::size_t>();

    const std::vector<std::string> tracked_body_names =
        yaml["motion"]["tracked_body_names"].as<std::vector<std::string>>();
    config.tracked_body_count = tracked_body_names.size();
    const std::string anchor_body_name =
        yaml["motion"]["anchor_body_name"].as<std::string>();
    const auto anchor_body =
        std::find(
            tracked_body_names.begin(), tracked_body_names.end(),
            anchor_body_name);
    if (anchor_body == tracked_body_names.end())
    {
        throw std::runtime_error(
            "RedRed anchor body is missing from motion.tracked_body_names");
    }
    config.anchor_body_index =
        static_cast<std::size_t>(
            std::distance(tracked_body_names.begin(), anchor_body));

    const std::vector<std::string> joint_names =
        yaml["policy_joint_names"].as<std::vector<std::string>>();
    if (joint_names.size() != kNumMotors)
    {
        throw std::runtime_error("policy_joint_names must contain 29 names");
    }
    config.joints.joint_to_motor =
        LoadArray<int, kNumMotors>(yaml, "joint2motor_idx");
    config.joints.default_angles =
        LoadArray<float, kNumMotors>(yaml, "default_angles");
    config.joints.kps = LoadArray<float, kNumMotors>(yaml, "kps");
    config.joints.kds = LoadArray<float, kNumMotors>(yaml, "kds");
    config.motion_frame0_joint_angles =
        LoadArray<float, kNumMotors>(yaml, "motion_frame0_joint_angles");
    config.action_scale = LoadArray<float, kNumMotors>(yaml, "action_scale");
    ValidateJointMapping(config.joints.joint_to_motor);
    ValidateUpperBodyMotorIndices(config.upper_body_motor_indices);
    config.waist_motor_indices = {
        FindJointMotorIndex(joint_names, config.joints.joint_to_motor, "waist_yaw_joint"),
        FindJointMotorIndex(joint_names, config.joints.joint_to_motor, "waist_roll_joint"),
        FindJointMotorIndex(joint_names, config.joints.joint_to_motor, "waist_pitch_joint")};

    int observation_size = 0;
    for (const YAML::Node &component : yaml["observation_layout"])
    {
        observation_size += component["dimension"].as<int>();
    }
    if (config.control_dt <= 0.0f || config.cast_duration <= 0.0f ||
        config.return_duration <= 0.0f ||
        !std::isfinite(config.cast_duration) ||
        !std::isfinite(config.return_duration) ||
        config.motion_fps <= 0.0f || config.expected_frames == 0 ||
        config.tracked_body_count == 0 ||
        std::abs(config.control_dt * config.motion_fps - 1.0f) > 1.0e-5f ||
        !std::all_of(
            config.motion_frame0_joint_angles.begin(),
            config.motion_frame0_joint_angles.end(),
            [](float value) { return std::isfinite(value); }) ||
        config.num_actions != 29 ||
        config.num_observations != 154 || observation_size != config.num_observations)
    {
        throw std::runtime_error("RedRed requires a 20 ms, 154-observation, 29-action contract");
    }
    return config;
}

} // namespace deploy_real
