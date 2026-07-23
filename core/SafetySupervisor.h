#ifndef DEPLOY_REAL_CORE_SAFETY_SUPERVISOR_H
#define DEPLOY_REAL_CORE_SAFETY_SUPERVISOR_H

#include "core/ControlTypes.h"

#include <optional>
#include <string>

namespace deploy_real
{

class SafetySupervisor
{
public:
    std::optional<std::string> CheckState(
        const StateContext &context, StateId current_state) const;
    bool IsCommandValid(const MotorCommand &command) const;
};

} // namespace deploy_real

#endif
