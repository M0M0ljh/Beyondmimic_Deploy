#ifndef DEPLOY_REAL_HARDWARE_UNITREE_JOYSTICK_H
#define DEPLOY_REAL_HARDWARE_UNITREE_JOYSTICK_H

#include <cstdint>

namespace deploy_real
{

union KeySwitch
{
    struct
    {
        std::uint8_t R1 : 1;
        std::uint8_t L1 : 1;
        std::uint8_t start : 1;
        std::uint8_t select : 1;
        std::uint8_t R2 : 1;
        std::uint8_t L2 : 1;
        std::uint8_t F1 : 1;
        std::uint8_t F2 : 1;
        std::uint8_t A : 1;
        std::uint8_t B : 1;
        std::uint8_t X : 1;
        std::uint8_t Y : 1;
        std::uint8_t up : 1;
        std::uint8_t right : 1;
        std::uint8_t down : 1;
        std::uint8_t left : 1;
    } components;
    std::uint16_t value;
};

struct JoystickData
{
    std::uint8_t head[2];
    KeySwitch buttons;
    float lx;
    float rx;
    float ry;
    float L2;
    float ly;
    std::uint8_t idle[16];
};

} // namespace deploy_real

#endif
