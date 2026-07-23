#ifndef DEPLOY_REAL_CORE_JOINT_PROFILE_H
#define DEPLOY_REAL_CORE_JOINT_PROFILE_H

#include "core/ControlTypes.h"

#include <array>

namespace deploy_real
{

struct JointProfile
{
    std::array<int, kNumMotors> joint_to_motor{};
    MotorArray default_angles{};
    MotorArray kps{};
    MotorArray kds{};
};

} // namespace deploy_real

#endif
