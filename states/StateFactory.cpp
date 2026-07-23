#include "states/StateFactory.h"

#include "states/basic/BasicStates.h"
#include "states/loco/LocoState.h"
#include "states/redred/RedRedCastState.h"
#include "states/redred/RedRedState.h"
#include "states/startup/StartupReadyState.h"

#include <memory>

namespace deploy_real
{

std::unique_ptr<StateMachine> CreateDefaultStateMachine(
    const LocoConfig &loco_config, const RedRedConfig &redred_config)
{
    auto state_machine = std::make_unique<StateMachine>();
    state_machine->RegisterState(std::make_unique<ZeroTorqueState>());
    state_machine->RegisterState(
        std::make_unique<StartupReadyState>(loco_config.joints));
    state_machine->RegisterState(std::make_unique<LocoState>(
        loco_config, redred_config.return_duration));
    state_machine->RegisterState(
        std::make_unique<RedRedCastState>(loco_config, redred_config));
    state_machine->RegisterState(std::make_unique<RedRedState>(redred_config));
    state_machine->RegisterState(std::make_unique<DampingState>());

    state_machine->AllowTransition(StateId::ZeroTorque, StateId::StartupReady);
    state_machine->AllowTransition(StateId::ZeroTorque, StateId::Damping);
    state_machine->AllowTransition(StateId::StartupReady, StateId::ZeroTorque);
    state_machine->AllowTransition(StateId::StartupReady, StateId::Loco);
    state_machine->AllowTransition(StateId::StartupReady, StateId::Damping);
    state_machine->AllowTransition(StateId::Loco, StateId::StartupReady);
    state_machine->AllowTransition(StateId::Loco, StateId::RedRedCast);
    state_machine->AllowTransition(StateId::Loco, StateId::Damping);
    state_machine->AllowTransition(StateId::RedRedCast, StateId::Loco);
    state_machine->AllowTransition(StateId::RedRedCast, StateId::RedRed);
    state_machine->AllowTransition(StateId::RedRedCast, StateId::Damping);
    state_machine->AllowTransition(StateId::RedRed, StateId::Loco);
    state_machine->AllowTransition(StateId::RedRed, StateId::StartupReady);
    state_machine->AllowTransition(StateId::RedRed, StateId::Damping);
    state_machine->AllowTransition(StateId::Damping, StateId::ZeroTorque);
    state_machine->SetInitialState(StateId::ZeroTorque);
    return state_machine;
}

} // namespace deploy_real
