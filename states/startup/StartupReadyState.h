#ifndef DEPLOY_REAL_STATES_STARTUP_STARTUP_READY_STATE_H
#define DEPLOY_REAL_STATES_STARTUP_STARTUP_READY_STATE_H

#include "core/ControlState.h"
#include "core/JointProfile.h"

#include <chrono>

namespace deploy_real
{

class StartupReadyState final : public ControlState
{
public:
    explicit StartupReadyState(JointProfile profile);

    StateId Id() const override;
    const char *Name() const override;
    void Enter(const StateContext &context) override;
    StateRequest Update(const StateContext &context, MotorCommand &command) override;
    void Exit(const StateContext &context) override;
    bool CanTransitionTo(StateId next) const override;

private:
    JointProfile profile_;
    MotorArray initial_position_{};
    std::chrono::steady_clock::time_point start_time_{};
    bool ready_{false};
};

} // namespace deploy_real

#endif
