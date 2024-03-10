#ifndef __HELPERS_HPP__
#define __HELPERS_HPP__

#include "rover_can_lib/rover_can_lib.hpp"

#if !defined(ESP32)
#include "rovus_lib/macros.h"
#endif // !defined(ESP32)

namespace RoverCanLib::Helpers
{
    template <typename COPY_TYPE, typename UNION_TYPE>
    void canMsgToStruct(IN const void *src_, OUT COPY_TYPE *dest_)
        requires (std::is_same_v<COPY_TYPE, float>     && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::FloatUnion>)    ||
                (std::is_same_v<COPY_TYPE, bool>       && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::BoolUnion>)     ||
                (std::is_same_v<COPY_TYPE, uint8_t>    && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Uint8_tUnion>)  ||
                (std::is_same_v<COPY_TYPE, uint16_t>   && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Uint16_tUnion>) ||
                (std::is_same_v<COPY_TYPE, uint32_t>   && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Uint32_tUnion>) ||
                (std::is_same_v<COPY_TYPE, int8_t>     && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Int8_tUnion>)   ||
                (std::is_same_v<COPY_TYPE, int16_t>    && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Int16_tUnion>)  ||
                (std::is_same_v<COPY_TYPE, int32_t>    && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Int32_tUnion>)
    {
        UNION_TYPE copyUnion;

        for (uint8_t i = 0; i < sizeof(copyUnion); i++)
        {
            copyUnion.dataBytes[i] = *((uint8_t *)src_ + i);
        }

        *dest_ = copyUnion.data;
    }
}

#endif // __HELPERS_HPP__
