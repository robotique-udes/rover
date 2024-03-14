#ifndef __CONFIG_HPP__
#define __CONFIG_HPP__

#include <cstdint>

namespace RoverCanLib::Constant
{
    constexpr uint8_t HEARTBEAT_FREQ = 4;
    constexpr uint16_t WATCHDOG_TIMEOUT_MS = 500;
}

#endif // __CONFIG_HPP__
