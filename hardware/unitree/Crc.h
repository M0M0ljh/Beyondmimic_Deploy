#ifndef DEPLOY_REAL_HARDWARE_UNITREE_CRC_H
#define DEPLOY_REAL_HARDWARE_UNITREE_CRC_H

#include <cstdint>

namespace deploy_real
{

std::uint32_t Crc32Core(const std::uint32_t *data, std::uint32_t length);

} // namespace deploy_real

#endif
