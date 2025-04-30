#ifndef HELPERS_HPP
#define HELPERS_HPP

#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/log.hpp"
#include "rover_can2/constant.hpp"
#include "rover_can2/can_msg.hpp"

#include <cstdint>
#include <bit>

DEFINE_LOG_NODE(CanHelpers, Logger::eNodeState::OFF)

namespace RoverCan2::Helpers
{
    /**
     * @brief
     * @attention [WARNING] Internal use only
     */
    template<typename ROVER_MSG_CONTENT_TYPE>
    constexpr bool DATA_LENGTH_MATCHES_MSG_CONTENT(uint8_t dataLength_)
    {
        static_assert(!std::is_pointer_v<ROVER_MSG_CONTENT_TYPE>, "Msg data can't be a pointer type");
        static_assert(!std::is_reference_v<ROVER_MSG_CONTENT_TYPE>, "Msg data can't be reference type");
        static_assert(!std::is_void_v<ROVER_MSG_CONTENT_TYPE>, "Msg data can't be void type");
        static_assert(!std::is_function_v<ROVER_MSG_CONTENT_TYPE>, "Msg data can't be a func pointer type");

        LOG_DEBUG(Logger::Nodes::CanHelpers,
                  "DATA_LENGTH_MATCHES_MSG_CONTENT(dataLength_ = %u): dataLength_ == (sizeof(ROVER_MSG_CONTENT_TYPE) - "
                  "TO_UNDERLYING(Constant::eDataIndex::START_OF_DATA)) -> %u == %u - %u ?)",
                  dataLength_,
                  dataLength_,
                  sizeof(ROVER_MSG_CONTENT_TYPE),
                  TO_UNDERLYING(Constant::eDataIndex::START_OF_DATA));
        return dataLength_ == (sizeof(ROVER_MSG_CONTENT_TYPE) + TO_UNDERLYING(Constant::eDataIndex::START_OF_DATA));
    }

    /**
     * @brief
     * @attention [WARNING] Internal use only
     */
    template<typename ROVER_MSG_CONTENT_TYPE>
    constexpr bool CAN_MSG_TO_ROVER_MSG_CONTENT(const CanMsg& canMsg_, ROVER_MSG_CONTENT_TYPE& msgContent_)
    {
        static_assert(!std::is_pointer_v<ROVER_MSG_CONTENT_TYPE>, "Msg data can't be a pointer type");
        static_assert(!std::is_reference_v<ROVER_MSG_CONTENT_TYPE>, "Msg data can't be reference type");
        static_assert(!std::is_void_v<ROVER_MSG_CONTENT_TYPE>, "Msg data can't be void type");
        static_assert(!std::is_function_v<ROVER_MSG_CONTENT_TYPE>, "Msg data can't be a func pointer type");

        static_assert(sizeof(ROVER_MSG_CONTENT_TYPE) <= (RoverCan2::Constant::CAN_MAX_DATA_LENGTH
                                                         - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA)));

        if (!DATA_LENGTH_MATCHES_MSG_CONTENT<ROVER_MSG_CONTENT_TYPE>(canMsg_.dataLength))
        {
            LOG_DEBUG(Logger::Nodes::CanHelpers, "DATA_LENGTH_MATCHES_MSG_CONTENT Failed");
            return false;
        }

        auto dataStartIndex = TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA);
        std::memcpy(&msgContent_, &(canMsg_.msgData[dataStartIndex]), sizeof(msgContent_));

        return true;
    }

    /**
     * @brief
     * @attention [WARNING] Internal use only
     */
    template<typename ROVER_MSG_CONTENT_TYPE>
    constexpr void ROVER_MSG_CONTENT_TO_CAN_MSG(RoverCan2::Constant::eMsgId msgId_,
                                                uint8_t msgContentID_,
                                                const ROVER_MSG_CONTENT_TYPE& msgContent_,
                                                CanMsg& canMsg_)
    {
        static_assert(!std::is_pointer_v<ROVER_MSG_CONTENT_TYPE>, "Msg data can't be a pointer type");
        static_assert(!std::is_reference_v<ROVER_MSG_CONTENT_TYPE>, "Msg data can't be reference type");
        static_assert(!std::is_void_v<ROVER_MSG_CONTENT_TYPE>, "Msg data can't be void type");
        static_assert(!std::is_function_v<ROVER_MSG_CONTENT_TYPE>, "Msg data can't be a func pointer type");

        static_assert(sizeof(ROVER_MSG_CONTENT_TYPE) <= (RoverCan2::Constant::CAN_MAX_DATA_LENGTH
                                                         - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA)));

        canMsg_.setMsgID(msgId_);
        canMsg_.dataLength = sizeof(ROVER_MSG_CONTENT_TYPE) + TO_UNDERLYING(Constant::eDataIndex::START_OF_DATA);
        canMsg_.setMsgContentID(msgContentID_);

        std::memcpy(&(canMsg_.msgData[TO_UNDERLYING(Constant::eDataIndex::START_OF_DATA)]), &msgContent_, sizeof(msgContent_));
    }

    /**
     * @brief
     * @attention [WARNING] Internal use only
     */
    template<typename ROVER_MSG_ENUM_TYPE>
    constexpr bool MSG_CONTENT_IS_LAST_ELEM(const CanMsg& canMsg_)
    {
        static_assert(std::is_enum_v<ROVER_MSG_ENUM_TYPE>, "Template argument most be of enum type");

        return canMsg_.msgData[(size_t)RoverCan2::Constant::eDataIndex::MSG_CONTENT_ID]
               == ((size_t)ROVER_MSG_ENUM_TYPE::eLAST - 1);
    }
}  // namespace RoverCan2::Helpers

#endif  // HELPERS_HPP
