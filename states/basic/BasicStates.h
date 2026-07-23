#ifndef DEPLOY_REAL_STATES_BASIC_BASIC_STATES_H
#define DEPLOY_REAL_STATES_BASIC_BASIC_STATES_H

#include "core/ControlState.h"

namespace deploy_real
{

class ZeroTorqueState final : public ControlState
{
public:
    StateId Id() const override;
    const char *Name() const override;
    void Enter(const StateContext &context) override;
    StateRequest Update(const StateContext &context, MotorCommand &command) override;
    void Exit(const StateContext &context) override;
};

class DampingState final : public ControlState
{
public:
    StateId Id() const override;
    const char *Name() const override;
    void Enter(const StateContext &context) override;
    StateRequest Update(const StateContext &context, MotorCommand &command) override;
    void Exit(const StateContext &context) override;
};

} // namespace deploy_real

#endif
