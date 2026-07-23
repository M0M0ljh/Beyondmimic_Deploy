#ifndef DEPLOY_REAL_STATES_STATE_FACTORY_H
#define DEPLOY_REAL_STATES_STATE_FACTORY_H

#include "core/StateMachine.h"
#include "states/loco/LocoConfig.h"
#include "states/redred/RedRedConfig.h"

#include <memory>

namespace deploy_real
{

std::unique_ptr<StateMachine> CreateDefaultStateMachine(
    const LocoConfig &loco_config, const RedRedConfig &redred_config);

} // namespace deploy_real

#endif
