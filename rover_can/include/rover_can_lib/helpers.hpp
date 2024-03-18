#ifndef __HELPERS_HPP__
#define __HELPERS_HPP__

#include <type_traits>
#include "rover_can_lib/msgs/error_state.hpp"
#include "rover_can_lib/union_type_definition.hpp"

#if defined(ESP32)
#include "driver/twai.h"
#include "helpers/macros.hpp"
#elif defined(__linux__) // defined(ESP32)
#include <linux/can.h>
#include "rovus_lib/macros.h"
#include "rclcpp/rclcpp.hpp"
#endif // defined(ESP32)

namespace RoverCanLib::Helpers
{
    // Return a msg with ID 0x00 which should be never be considered by any
    // nodes, it's used internally to say that theres no more message in the rx
    // queue
#if defined(ESP32)
    inline twai_message_t getErrorIdMsg(void)
    {
        twai_message_t msg;
        msg.identifier = 0u;

        return msg;
    }

    template <typename COPY_TYPE, typename UNION_TYPE>
    void canMsgToStruct(IN const twai_message_t *msg_, OUT COPY_TYPE *dest_)
        requires(std::is_same_v<COPY_TYPE, float> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::FloatUnion>) ||
                (std::is_same_v<COPY_TYPE, bool> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::BoolUnion>) ||
                (std::is_same_v<COPY_TYPE, uint8_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Uint8_tUnion>) ||
                (std::is_same_v<COPY_TYPE, uint16_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Uint16_tUnion>) ||
                (std::is_same_v<COPY_TYPE, uint32_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Uint32_tUnion>) ||
                (std::is_same_v<COPY_TYPE, int8_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Int8_tUnion>) ||
                (std::is_same_v<COPY_TYPE, int16_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Int16_tUnion>) ||
                (std::is_same_v<COPY_TYPE, int32_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Int32_tUnion>)
    {
        if (msg_->data_length_code < ((uint8_t)Constant::eDataIndex::START_OF_DATA + sizeof(COPY_TYPE)))
        {
            LOG(WARN, "Wrong length for data type, dropping msg");
            return;
        }

        UNION_TYPE copyUnion;

        for (uint8_t i = 0; i < sizeof(copyUnion); i++)
        {
            copyUnion.dataBytes[i] = *((uint8_t *)msg_->data + (uint8_t)Constant::eDataIndex::START_OF_DATA + i);
        }

        *dest_ = copyUnion.data;
    }

    template <typename COPY_TYPE, typename UNION_TYPE>
    void structToCanMsg(IN const COPY_TYPE *structMember_, OUT twai_message_t *msg_)
        requires(std::is_same_v<COPY_TYPE, float> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::FloatUnion>) ||
                (std::is_same_v<COPY_TYPE, bool> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::BoolUnion>) ||
                (std::is_same_v<COPY_TYPE, uint8_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Uint8_tUnion>) ||
                (std::is_same_v<COPY_TYPE, uint16_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Uint16_tUnion>) ||
                (std::is_same_v<COPY_TYPE, uint32_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Uint32_tUnion>) ||
                (std::is_same_v<COPY_TYPE, int8_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Int8_tUnion>) ||
                (std::is_same_v<COPY_TYPE, int16_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Int16_tUnion>) ||
                (std::is_same_v<COPY_TYPE, int32_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Int32_tUnion>)
    {
        msg_->data_length_code = ((uint8_t)Constant::eDataIndex::START_OF_DATA + sizeof(COPY_TYPE));

        UNION_TYPE copyUnion;
        copyUnion.data = *structMember_;

        for (uint8_t i = 0; i < sizeof(copyUnion); i++)
        {
            *((uint8_t *)msg_->data + (uint8_t)Constant::eDataIndex::START_OF_DATA + i) = copyUnion.dataBytes[i];
        }
    }

#elif defined(__linux__) // defined(ESP32)
    can_frame getErrorIdMsg(void)
    {
        can_frame msg;
        msg.can_id = 0u;

        return msg;
    }

    template <typename COPY_TYPE, typename UNION_TYPE>
    void canMsgToStruct(IN const can_frame *msg_, OUT COPY_TYPE *dest_, IN rclcpp::Logger logger_)
        requires(std::is_same_v<COPY_TYPE, float> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::FloatUnion>) ||
                (std::is_same_v<COPY_TYPE, bool> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::BoolUnion>) ||
                (std::is_same_v<COPY_TYPE, uint8_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Uint8_tUnion>) ||
                (std::is_same_v<COPY_TYPE, uint16_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Uint16_tUnion>) ||
                (std::is_same_v<COPY_TYPE, uint32_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Uint32_tUnion>) ||
                (std::is_same_v<COPY_TYPE, int8_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Int8_tUnion>) ||
                (std::is_same_v<COPY_TYPE, int16_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Int16_tUnion>) ||
                (std::is_same_v<COPY_TYPE, int32_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Int32_tUnion>)
    {
        if ((uint8_t)msg_->len < (uint8_t)Constant::eDataIndex::START_OF_DATA + sizeof(COPY_TYPE))
        {
            RCLCPP_WARN(logger_, "Wrong length for data type, dropping msg");
            return;
        }

        UNION_TYPE copyUnion;

        for (uint8_t i = 0; i < sizeof(copyUnion); i++)
        {
            copyUnion.dataBytes[i] = *((uint8_t *)msg_->data + (uint8_t)Constant::eDataIndex::START_OF_DATA + i);
        }

        *dest_ = copyUnion.data;
    }

    template <typename COPY_TYPE, typename UNION_TYPE>
    void structToCanMsg(IN const COPY_TYPE *structMember_, OUT can_frame *msg_)
        requires(std::is_same_v<COPY_TYPE, float> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::FloatUnion>) ||
                (std::is_same_v<COPY_TYPE, bool> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::BoolUnion>) ||
                (std::is_same_v<COPY_TYPE, uint8_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Uint8_tUnion>) ||
                (std::is_same_v<COPY_TYPE, uint16_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Uint16_tUnion>) ||
                (std::is_same_v<COPY_TYPE, uint32_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Uint32_tUnion>) ||
                (std::is_same_v<COPY_TYPE, int8_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Int8_tUnion>) ||
                (std::is_same_v<COPY_TYPE, int16_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Int16_tUnion>) ||
                (std::is_same_v<COPY_TYPE, int32_t> && std::is_same_v<UNION_TYPE, RoverCanLib::UnionDefinition::Int32_tUnion>)
    {
        msg_->len = ((uint8_t)Constant::eDataIndex::START_OF_DATA + sizeof(COPY_TYPE));

        UNION_TYPE copyUnion;
        copyUnion.data = *structMember_;

        for (uint8_t i = 0; i < sizeof(copyUnion); i++)
        {
            *((uint8_t *)msg_->data + (uint8_t)Constant::eDataIndex::START_OF_DATA + i) = copyUnion.dataBytes[i];
        }
    }
#endif                   // defined(ESP32)
}

#endif                   // __HELPERS_HPP__
