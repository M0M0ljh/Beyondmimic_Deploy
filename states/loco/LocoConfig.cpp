#include "states/loco/LocoConfig.h"

#include "common/ProjectPath.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
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

} // namespace

LocoConfig LocoConfig::Load(const std::string &config_path)
{
    const YAML::Node yaml = YAML::LoadFile(config_path);
    LocoConfig config;
    config.control_dt = yaml["control_dt"].as<float>();
    config.model_path = ProjectPath(yaml["policy_path"].as<std::string>());
    config.joints.joint_to_motor =
        LoadArray<int, kNumMotors>(yaml, "joint2motor_idx");
    config.joints.default_angles =
        LoadArray<float, kNumMotors>(yaml, "default_angles");
    config.joints.kps = LoadArray<float, kNumMotors>(yaml, "kps");
    config.joints.kds = LoadArray<float, kNumMotors>(yaml, "kds");
    config.angular_velocity_scale = yaml["ang_vel_scale"].as<float>();
    config.joint_position_scale = yaml["dof_pos_scale"].as<float>();
    config.joint_velocity_scale = yaml["dof_vel_scale"].as<float>();
    config.action_scale = yaml["action_scale"].as<float>();
    config.command_scale = LoadArray<float, 3>(yaml, "cmd_scale");
    config.maximum_command = LoadArray<float, 3>(yaml, "max_cmd");
    config.planar_command_dead_zone = yaml["planar_command_dead_zone"].as<float>();
    config.planar_command_smoothing =
        yaml["planar_command_smoothing"].as<float>();
    config.num_actions = yaml["num_actions"].as<int>();
    config.num_observations = yaml["num_obs"].as<int>();

    ValidateJointMapping(config.joints.joint_to_motor);
    if (config.control_dt <= 0.0f ||
        !std::isfinite(config.planar_command_dead_zone) ||
        config.planar_command_dead_zone < 0.0f ||
        config.planar_command_dead_zone >= 1.0f ||
        !std::isfinite(config.planar_command_smoothing) ||
        config.planar_command_smoothing < 0.0f ||
        config.planar_command_smoothing > 1.0f)
    {
        throw std::runtime_error(
            "Loco control_dt, planar command dead zone, or smoothing is invalid");
    }
    constexpr int kLocoObservationSize = 9 + 3 * static_cast<int>(kNumMotors);
    if (config.num_actions != static_cast<int>(kNumMotors) ||
        config.num_observations != kLocoObservationSize)
    {
        throw std::runtime_error("Loco requires 29 actions and 96 observations");
    }
    return config;
}

} // namespace deploy_real
