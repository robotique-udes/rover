#ifndef __UNION_TYPE_DEFINITION__
#define __UNION_TYPE_DEFINITION__

#include <cstdint>

namespace RoverCanLib::UnionDefinition
{
    union FloatUnion
    {
        float data;
        uint8_t dataBytes[sizeof(data)];
    };

    union BoolUnion
    {
        bool data;
        uint8_t dataBytes[sizeof(data)];
    };

    union Uint8_tUnion
    {
        uint8_t data;
        uint8_t dataBytes[sizeof(data)];
    };

    union Uint16_tUnion
    {
        uint16_t data;
        uint8_t dataBytes[sizeof(data)];
    };

    union Uint32_tUnion
    {
        uint32_t data;
        uint8_t dataBytes[sizeof(data)];
    };

    union Int8_tUnion
    {
        int8_t data;
        uint8_t dataBytes[sizeof(data)];
    };

    union Int16_tUnion
    {
        int16_t data;
        uint8_t dataBytes[sizeof(data)];
    };

    union Int32_tUnion
    {
        int32_t data;
        uint8_t dataBytes[sizeof(data)];
    };

}

#endif // __UNION_TYPE_DEFINITION__
