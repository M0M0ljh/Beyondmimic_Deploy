#ifndef DEPLOY_REAL_STATES_POLICY_POLICY_STATE_H
#define DEPLOY_REAL_STATES_POLICY_POLICY_STATE_H

#include "core/ControlState.h"
#include "inference/OnnxRuntime.h"

#include <string>

namespace deploy_real
{

class PolicyState : public ControlState
{
public:
    PolicyState(StateId id, std::string name, const std::string &model_path);

    StateId Id() const final;
    const char *Name() const final;
    void Enter(const StateContext &context) final;
    StateRequest Update(const StateContext &context, MotorCommand &command) final;
    void Exit(const StateContext &context) final;

protected:
    OnnxRuntime &Runtime();
    const OnnxRuntime &Runtime() const;

    virtual void OnEnter(const StateContext &) {}
    virtual TensorMap BuildInputs(const StateContext &context) = 0;
    virtual void DecodeOutputs(
        const StateContext &context, const TensorMap &outputs,
        MotorCommand &command) = 0;
    virtual StateRequest AfterInference(const StateContext &)
    {
        return StateRequest::None();
    }
    virtual void OnExit(const StateContext &) {}

private:
    StateId id_;
    std::string name_;
    OnnxRuntime runtime_;
};

} // namespace deploy_real

#endif
