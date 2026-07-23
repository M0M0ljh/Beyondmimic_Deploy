#include "core/ControlTypes.h"

namespace deploy_real
{

const char *ToString(StateId state)
{
    switch (state)
    {
    case StateId::ZeroTorque:
        return "ZeroTorque";
    case StateId::StartupReady:
        return "StartupReady";
    case StateId::Loco:
        return "Loco";
    case StateId::RedRedCast:
        return "RedRedCast";
    case StateId::RedRed:
        return "RedRed";
    case StateId::Damping:
        return "Damping";
    }
    return "Unknown";
}

} // namespace deploy_real
