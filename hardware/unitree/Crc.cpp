#include "hardware/unitree/Crc.h"

namespace deploy_real
{

std::uint32_t Crc32Core(const std::uint32_t *data, std::uint32_t length)
{
    std::uint32_t crc = 0xFFFFFFFF;
    constexpr std::uint32_t polynomial = 0x04c11db7;
    for (std::uint32_t i = 0; i < length; ++i)
    {
        std::uint32_t xbit = 1U << 31;
        const std::uint32_t value = data[i];
        for (std::uint32_t bit = 0; bit < 32; ++bit)
        {
            if (crc & 0x80000000)
            {
                crc = (crc << 1) ^ polynomial;
            }
            else
            {
                crc <<= 1;
            }
            if (value & xbit)
            {
                crc ^= polynomial;
            }
            xbit >>= 1;
        }
    }
    return crc;
}

} // namespace deploy_real
