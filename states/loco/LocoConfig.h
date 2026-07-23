#ifndef DEPLOY_REAL_STATES_LOCO_LOCO_CONFIG_H
#define DEPLOY_REAL_STATES_LOCO_LOCO_CONFIG_H

#include "core/JointProfile.h"

#include <array>
#include <string>

namespace deploy_real
{

struct LocoConfig
{
    static LocoConfig Load(const std::string &config_path);

    float control_dt{0.02f};
    std::string model_path;
    JointProfile joints;
    float angular_velocity_scale{1.0f};
    float joint_position_scale{1.0f};
    float joint_velocity_scale{1.0f};
    float action_scale{1.0f};
    std::array<float, 3> command_scale{};
    std::array<float, 3> maximum_command{};
    float planar_command_dead_zone{0.1f};
    float planar_command_smoothing{0.95f};
    int num_actions{0};
    int num_observations{0};
};

} // namespace deploy_real

#endif
