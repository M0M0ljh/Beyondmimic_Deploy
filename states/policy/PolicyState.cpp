#include "states/policy/PolicyState.h"

#include <utility>

namespace deploy_real
{

PolicyState::PolicyState(StateId id, std::string name, const std::string &model_path)
    : id_(id), name_(std::move(name)), runtime_(model_path)
{
}

StateId PolicyState::Id() const
{
    return id_;
}

const char *PolicyState::Name() const
{
    return name_.c_str();
}

void PolicyState::Enter(const StateContext &context)
{
    OnEnter(context);
}

StateRequest PolicyState::Update(
    const StateContext &context, MotorCommand &command)
{
    command = {};
    TensorMap inputs = BuildInputs(context);
    const TensorMap outputs = runtime_.Run(std::move(inputs));
    DecodeOutputs(context, outputs, command);
    return AfterInference(context);
}

void PolicyState::Exit(const StateContext &context)
{
    OnExit(context);
}

OnnxRuntime &PolicyState::Runtime()
{
    return runtime_;
}

const OnnxRuntime &PolicyState::Runtime() const
{
    return runtime_;
}

} // namespace deploy_real
