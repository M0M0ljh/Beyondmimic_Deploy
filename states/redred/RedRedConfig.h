#ifndef DEPLOY_REAL_STATES_REDRED_REDRED_CONFIG_H
#define DEPLOY_REAL_STATES_REDRED_REDRED_CONFIG_H

#include "core/JointProfile.h"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace deploy_real
{

struct RedRedConfig
{
    static RedRedConfig Load(const std::string &config_path);

    float control_dt{0.02f};
    std::string model_path;
    int num_observations{0};
    int num_actions{0};
    std::string observation_input_name;
    std::string time_step_input_name;
    std::string action_output_name;
    std::string reference_joint_position_output_name;
    std::string reference_joint_velocity_output_name;
    std::string reference_body_orientation_output_name;
    bool loop_motion{false};
    StateId end_state{StateId::Loco};
    float cast_duration{0.1f};
    float return_duration{0.1f};
    float motion_fps{50.0f};
    std::vector<std::size_t> upper_body_motor_indices;
    std::size_t expected_frames{0};
    std::size_t tracked_body_count{0};
    std::size_t anchor_body_index{0};
    std::array<std::size_t, 3> waist_motor_indices{};
    JointProfile joints;
    MotorArray motion_frame0_joint_angles{};
    MotorArray action_scale{};
};

} // namespace deploy_real

#endif
