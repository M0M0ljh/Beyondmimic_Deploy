#ifndef DEPLOY_REAL_CORE_STATE_MACHINE_H
#define DEPLOY_REAL_CORE_STATE_MACHINE_H

#include "core/ControlState.h"
#include "core/SafetySupervisor.h"

#include <map>
#include <memory>
#include <set>
#include <string>

namespace deploy_real
{

class StateMachine
{
public:
    void RegisterState(std::unique_ptr<ControlState> state);
    void AllowTransition(StateId from, StateId to);
    void SetInitialState(StateId state);

    void Update(const StateContext &context, MotorCommand &command);
    StateId CurrentState() const;
    const std::string &LatchedFault() const;

private:
    bool TryTransition(StateId next, const StateContext &context, const std::string &reason);
    void TransitionTo(StateId next, const StateContext &context, const std::string &reason);
    void EnterDamping(const StateContext &context, const std::string &reason);
    void UpdateCurrentState(const StateContext &context, MotorCommand &command);

    std::map<StateId, std::unique_ptr<ControlState>> states_;
    std::map<StateId, std::set<StateId>> allowed_transitions_;
    StateId initial_state_{StateId::ZeroTorque};
    ControlState *current_state_{nullptr};
    SafetySupervisor safety_;
    std::string latched_fault_;
};

} // namespace deploy_real

#endif
