#ifndef __BASE_OBJECTS_HPP__
#define __BASE_OBJECTS_HPP__

#include <stdint.h>

namespace RoverRosSerial
{
    namespace Constant
    {
        constexpr uint8_t BEGIN = 128u;

        enum eHeaderType : uint16_t
        {
            notInitialised = 0u,
            heartbeat = 129u,
            log = 130u,
            Gps,
            AntennaCmd,
        };

        struct sHeader
        {
            uint16_t type;
            uint16_t length;
        };

        union uHeader
        {
            sHeader header;
            uint8_t data[sizeof(sHeader)];
        };
    }
}
#endif // __BASE_OBJECTS_HPP__
