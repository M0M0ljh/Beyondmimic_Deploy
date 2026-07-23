#include "states/basic/BasicStates.h"

namespace deploy_real
{
namespace
{

constexpr float kDampingKd = 8.0f;

} // namespace

StateId ZeroTorqueState::Id() const
{
    return StateId::ZeroTorque;
}

const char *ZeroTorqueState::Name() const
{
    return ToString(Id());
}

void ZeroTorqueState::Enter(const StateContext &) {}

StateRequest ZeroTorqueState::Update(const StateContext &, MotorCommand &command)
{
    command = {};
    return StateRequest::None();
}

void ZeroTorqueState::Exit(const StateContext &) {}

StateId DampingState::Id() const
{
    return StateId::Damping;
}

const char *DampingState::Name() const
{
    return ToString(Id());
}

void DampingState::Enter(const StateContext &) {}

StateRequest DampingState::Update(const StateContext &, MotorCommand &command)
{
    command = {};
    command.kd.fill(kDampingKd);
    return StateRequest::None();
}

void DampingState::Exit(const StateContext &) {}

} // namespace deploy_real
