#include "core/StateMachine.h"

#include <exception>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace deploy_real
{

void StateMachine::RegisterState(std::unique_ptr<ControlState> state)
{
    const StateId id = state->Id();
    if (!states_.emplace(id, std::move(state)).second)
    {
        throw std::runtime_error("duplicate state registration");
    }
}

void StateMachine::AllowTransition(StateId from, StateId to)
{
    allowed_transitions_[from].insert(to);
}

void StateMachine::SetInitialState(StateId state)
{
    if (current_state_)
    {
        throw std::runtime_error("initial state cannot change after the FSM has started");
    }
    if (states_.count(state) == 0)
    {
        throw std::runtime_error("initial state is not registered");
    }
    initial_state_ = state;
}

bool StateMachine::TryTransition(
    StateId next, const StateContext &context, const std::string &reason)
{
    if (current_state_->Id() == next)
    {
        return true;
    }
    const auto allowed = allowed_transitions_.find(current_state_->Id());
    if (allowed == allowed_transitions_.end() || allowed->second.count(next) == 0)
    {
        std::cerr << "[FSM] rejected transition " << current_state_->Name()
                  << " -> " << ToString(next) << '\n';
        return false;
    }
    if (!current_state_->CanTransitionTo(next))
    {
        std::cerr << "[FSM] " << current_state_->Name()
                  << " is not ready for " << ToString(next) << '\n';
        return false;
    }
    TransitionTo(next, context, reason);
    return true;
}

void StateMachine::TransitionTo(
    StateId next, const StateContext &context, const std::string &reason)
{
    const auto next_state = states_.find(next);
    if (next_state == states_.end())
    {
        throw std::runtime_error("requested state is not registered");
    }

    const char *previous_name = current_state_ ? current_state_->Name() : nullptr;
    if (current_state_)
    {
        current_state_->Exit(context);
    }
    current_state_ = next_state->second.get();
    current_state_->Enter(context);

    std::cout << "[FSM] ";
    if (previous_name)
    {
        std::cout << previous_name << " -> ";
    }
    std::cout << current_state_->Name();
    if (!reason.empty())
    {
        std::cout << " (" << reason << ")";
    }
    std::cout << '\n';
}

void StateMachine::EnterDamping(const StateContext &context, const std::string &reason)
{
    latched_fault_ = reason;
    if (states_.count(StateId::Damping) == 0)
    {
        throw std::runtime_error("Damping state must be registered for safety fallback");
    }
    if (!current_state_ || current_state_->Id() != StateId::Damping)
    {
        TransitionTo(StateId::Damping, context, reason);
    }
}

void StateMachine::UpdateCurrentState(
    const StateContext &context, MotorCommand &command)
{
    const StateRequest request = current_state_->Update(context, command);
    if (request.next_state)
    {
        TryTransition(*request.next_state, context, request.reason);
    }
}

void StateMachine::Update(const StateContext &context, MotorCommand &command)
{
    if (!current_state_)
    {
        TransitionTo(initial_state_, context, "initial state");
    }

    if (const auto fault = safety_.CheckState(context, current_state_->Id()))
    {
        EnterDamping(context, *fault);
    }
    else if (context.input.requested_state)
    {
        if (TryTransition(*context.input.requested_state, context, "operator request") &&
            current_state_->Id() == StateId::ZeroTorque)
        {
            latched_fault_.clear();
        }
    }

    try
    {
        UpdateCurrentState(context, command);
        if (!safety_.IsCommandValid(command))
        {
            throw std::runtime_error("motor command contains a non-finite value");
        }
    }
    catch (const std::exception &error)
    {
        EnterDamping(context, error.what());
        current_state_->Update(context, command);
    }
}

StateId StateMachine::CurrentState() const
{
    return current_state_ ? current_state_->Id() : initial_state_;
}

const std::string &StateMachine::LatchedFault() const
{
    return latched_fault_;
}

} // namespace deploy_real
