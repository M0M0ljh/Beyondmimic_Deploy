#ifndef DEPLOY_REAL_STATES_REDRED_REDRED_STATE_H
#define DEPLOY_REAL_STATES_REDRED_REDRED_STATE_H

#include "states/policy/PolicyState.h"
#include "states/redred/RedRedConfig.h"

#include <cstddef>
#include <vector>

namespace deploy_real
{

class RedRedState final : public PolicyState
{
public:
    explicit RedRedState(RedRedConfig config);

protected:
    void OnEnter(const StateContext &context) override;
    TensorMap BuildInputs(const StateContext &context) override;
    void DecodeOutputs(
        const StateContext &context, const TensorMap &outputs,
        MotorCommand &command) override;
    StateRequest AfterInference(const StateContext &context) override;
    void OnExit(const StateContext &context) override;

private:
    void ValidateModelContract() const;
    TensorMap GenerateReferenceFrame(std::size_t frame_index);
    void CacheReferenceOutputs(const TensorMap &outputs);
    void ValidateInitialReference() const;
    std::size_t NextReferenceFrame() const;
    void UpdateProgress(std::size_t completed_frames);
    void FinishProgressLine();

    RedRedConfig config_;
    std::vector<float> reference_joint_position_;
    std::vector<float> reference_joint_velocity_;
    std::vector<float> reference_body_orientation_;
    std::vector<float> previous_action_;
    std::size_t frame_index_{0};
    float reference_yaw_offset_{0.0f};
    int last_progress_percent_{-1};
    bool progress_line_open_{false};
    bool motion_finished_{false};
};

} // namespace deploy_real

#endif
