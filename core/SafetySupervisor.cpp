#include "core/SafetySupervisor.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace deploy_real
{
namespace
{

constexpr auto kMaximumStateAge = std::chrono::milliseconds(100);

template <typename Container>
bool AllFinite(const Container &values)
{
    return std::all_of(values.begin(), values.end(), [](float value) {
        return std::isfinite(value);
    });
}

} // namespace

std::optional<std::string> SafetySupervisor::CheckState(
    const StateContext &context, StateId current_state) const
{
    if (!context.robot.valid)
    {
        return "robot state is unavailable";
    }
    if (context.robot.age > kMaximumStateAge)
    {
        return "robot state is stale";
    }
    if (!AllFinite(context.robot.position) || !AllFinite(context.robot.velocity) ||
        !AllFinite(context.robot.estimated_torque) ||
        !AllFinite(context.robot.base_quaternion) ||
        !AllFinite(context.robot.angular_velocity) || !AllFinite(context.robot.gravity))
    {
        return "robot state contains a non-finite value";
    }
    if (current_state != StateId::ZeroTorque && current_state != StateId::Damping &&
        context.robot.gravity[2] > 0.0f)
    {
        return "robot orientation is outside the safe range";
    }
    return std::nullopt;
}

bool SafetySupervisor::IsCommandValid(const MotorCommand &command) const
{
    return AllFinite(command.position) && AllFinite(command.velocity) &&
           AllFinite(command.kp) && AllFinite(command.kd) && AllFinite(command.torque);
}

} // namespace deploy_real
