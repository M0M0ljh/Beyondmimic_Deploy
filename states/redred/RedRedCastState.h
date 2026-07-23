#ifndef DEPLOY_REAL_STATES_REDRED_REDRED_CAST_STATE_H
#define DEPLOY_REAL_STATES_REDRED_REDRED_CAST_STATE_H

#include "states/loco/LocoState.h"
#include "states/redred/RedRedConfig.h"

#include <array>
#include <chrono>

namespace deploy_real
{

class RedRedCastState final : public LocoState
{
public:
    RedRedCastState(LocoConfig loco_config, RedRedConfig redred_config);

protected:
    void OnEnter(const StateContext &context) override;
    std::array<float, 3> CommandVelocity(
        const StateContext &context) override;
    void DecodeOutputs(
        const StateContext &context, const TensorMap &outputs,
        MotorCommand &command) override;
    StateRequest AfterInference(const StateContext &context) override;

private:
    float InterpolationPhase(const StateContext &context) const;

    RedRedConfig redred_config_;
    std::array<std::size_t, kNumMotors> motor_to_policy_{};
    MotorArray initial_position_{};
    std::chrono::steady_clock::time_point start_time_{};
};

} // namespace deploy_real

#endif
