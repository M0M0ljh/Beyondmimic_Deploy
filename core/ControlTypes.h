#ifndef DEPLOY_REAL_CORE_CONTROL_TYPES_H
#define DEPLOY_REAL_CORE_CONTROL_TYPES_H

#include <array>
#include <chrono>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>

namespace deploy_real
{

constexpr std::size_t kNumMotors = 29;
using MotorArray = std::array<float, kNumMotors>;

enum class StateId
{
    ZeroTorque,
    StartupReady,
    Loco,
    RedRedCast,
    RedRed,
    Damping,
};

const char *ToString(StateId state);

struct RobotState
{
    MotorArray position{};
    MotorArray velocity{};
    MotorArray estimated_torque{};
    std::array<float, 4> base_quaternion{};
    std::array<float, 3> angular_velocity{};
    std::array<float, 3> gravity{};
    bool valid{false};
    std::chrono::steady_clock::duration age{};
};

struct UserInput
{
    std::optional<StateId> requested_state;
    std::array<float, 3> velocity_command{};
    std::optional<int> skill_id;
    std::optional<std::string> motion_id;
    bool quit{false};
};

struct MotorCommand
{
    MotorArray position{};
    MotorArray velocity{};
    MotorArray kp{};
    MotorArray kd{};
    MotorArray torque{};
};

struct StateContext
{
    const RobotState &robot;
    const UserInput &input;
    std::chrono::steady_clock::time_point now;
    float control_dt;
};

struct StateRequest
{
    std::optional<StateId> next_state;
    std::string reason;

    static StateRequest None()
    {
        return {};
    }

    static StateRequest Transition(StateId next, std::string why = {})
    {
        return {next, std::move(why)};
    }
};

} // namespace deploy_real

#endif
