#ifndef DEPLOY_REAL_CORE_CONTROL_STATE_H
#define DEPLOY_REAL_CORE_CONTROL_STATE_H

#include "core/ControlTypes.h"

namespace deploy_real
{

class ControlState
{
public:
    virtual ~ControlState() = default;

    virtual StateId Id() const = 0;
    virtual const char *Name() const = 0;
    virtual void Enter(const StateContext &context) = 0;
    virtual StateRequest Update(const StateContext &context, MotorCommand &command) = 0;
    virtual void Exit(const StateContext &context) = 0;
    virtual bool CanTransitionTo(StateId) const
    {
        return true;
    }
};

} // namespace deploy_real

#endif
