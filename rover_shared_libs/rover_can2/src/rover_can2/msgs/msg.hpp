#ifndef ROVER_CAN2_MSGS_MSG_HPP
#define ROVER_CAN2_MSGS_MSG_HPP

#include "rover_can2/constant.hpp"
#include "rover_can2/can_msg.hpp"

#include <optional>

namespace RoverCan2::Msgs
{
    enum class eLoadMsgCode
    {
        SUCCESS_COMPLETE,
        SUCCESS_INCOMPLETE,
        NOT_CONCERNED,
        ERROR_MISMATCH,
        ERROR_IMPLEMENTATION,
        ERROR_INVALID_MSG
    };

    // Shadow for type validation
    class MsgBaseT
    {
      protected:
        MsgBaseT() = default;
    };

    template<typename ImplT>
    class Msg : public MsgBaseT
    {
        friend ImplT;

      public:
        constexpr RoverCan2::Constant::eMsgId getMsgId() const
        {
            return _msgID;
        }

        /**
         * @brief
         *
         * @param msg
         * @return eLoadMsgCode
         *    SUCCESS_COMPLETE: Successfully loaded the last element of a msg
         *    SUCCESS_INCOMPLETE: Successfully loaded any element of a msg other than the last
         *    NOT_CONCERNED: Msg ID doesn't match
         *    ERROR_MISMATCH: Mismatching message definition between the one received and the local on the device
         *    ERROR_IMPLEMENTATION: Shouldn't return this error code, it mean the message definition itself is erronous
         */
        eLoadMsgCode loadMsg(const CanMsg& canMsg_)
        {
            return static_cast<ImplT*>(this)->_loadMsg(canMsg_);
        }

        /**
         * @brief Return the message as a CanMsg
         *
         */
        std::optional<CanMsg> getCanMsg(const uint8_t msgContentId_) const
        {
            return static_cast<const ImplT*>(this)->_getCanMsg(msgContentId_);
        }

        /**
         * @brief Return the nb of elements in message
         *
         */
        uint8_t getMsgContentCount(void) const
        {
            return static_cast<const ImplT*>(this)->_getMsgContentCount();
        }

      private:
        Msg(RoverCan2::Constant::eMsgId msgId_):
            _msgID(msgId_)
        {
        }

        const RoverCan2::Constant::eMsgId _msgID;
    };
}  // namespace RoverCan2::Msgs

#endif  // ROVER_CAN2_MSGS_MSG_HPP
