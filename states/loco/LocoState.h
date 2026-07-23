#ifndef DEPLOY_REAL_STATES_LOCO_LOCO_STATE_H
#define DEPLOY_REAL_STATES_LOCO_LOCO_STATE_H

#include "states/loco/LocoConfig.h"
#include "states/policy/PolicyState.h"

#include <array>
#include <chrono>
#include <string>
#include <vector>

namespace deploy_real
{

class LocoState : public PolicyState
{
public:
    explicit LocoState(
        LocoConfig config, float entry_blend_duration = 0.0f);

protected:
    LocoState(
        StateId id, std::string name, LocoConfig config,
        float entry_blend_duration = 0.0f);

    void OnEnter(const StateContext &context) override;
    TensorMap BuildInputs(const StateContext &context) override;
    void DecodeOutputs(
        const StateContext &context, const TensorMap &outputs,
        MotorCommand &command) override;
    virtual std::array<float, 3> CommandVelocity(
        const StateContext &context);

private:
    void ValidateModelContract() const;
    float EntryBlendPhase(const StateContext &context) const;

    LocoConfig config_;
    std::vector<float> previous_action_;
    std::array<float, 2> previous_planar_command_{};
    float entry_blend_duration_{0.0f};
    MotorArray entry_position_{};
    std::chrono::steady_clock::time_point entry_start_time_{};
};

} // namespace deploy_real

#endif
