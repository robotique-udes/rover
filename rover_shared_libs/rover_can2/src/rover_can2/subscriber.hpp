#ifndef ROVER_CAN2_SUBSCRIBER_HPP
#define ROVER_CAN2_SUBSCRIBER_HPP

#include "rover_can2/msgs/msg.hpp"

#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/assert.hpp"

DEFINE_LOG_NODE(Subscriber, Logger::eNodeState::OFF)

namespace RoverCan2
{
    /**
     * @brief Base Subscriber class. Allows template abstraction for type validation
     */
    class SubscriberBaseT
    {
      protected:
        SubscriberBaseT() = default;
    };

    /**
     * @brief SubscriberBase CRTP Interface; Registering a subscriber to a DeviceT enables the reception of a certain CAN ID and
     * MsgT and links it's reception to a user specified callback for data handling.
     *
     * @attention [WARNING] Interface, can't be used directly
     *
     * @tparam ImplT
     * @tparam MsgT
     */
    template<typename ImplT, typename MsgT>
    class SubscriberBase : public SubscriberBaseT
    {
        VALIDATE_BASE_TYPE(Msgs::MsgBaseT, MsgT);

        friend ImplT;

      public:
        Msgs::eLoadMsgCode parseMsg(const CanMsg& msgCan_)
        {
            static_assert(std::is_base_of<Msgs::Msg<MsgT>, MsgT>::value,
                          "Subscriber's message type must be of base class RoverCan2::Msgs::Msg");

            Msgs::eLoadMsgCode loadCode = this->getMsg().loadMsg(msgCan_);
            switch (loadCode)
            {
                case Msgs::eLoadMsgCode::SUCCESS_COMPLETE:
                    LOG_DEBUG(Logger::Nodes::Subscriber, "Loaded last part of message, calling related callback");
                    this->triggerCallback(this->getMsg());
                    break;
                case Msgs::eLoadMsgCode::SUCCESS_INCOMPLETE:
                    LOG_DEBUG(Logger::Nodes::Subscriber, "Loaded a part of message, waiting on for the rest...");
                    break;
                case Msgs::eLoadMsgCode::NOT_CONCERNED:
                    LOG_DEBUG(Logger::Nodes::Subscriber, "Parsed msg wasn't meant for this subscriber");
                    break;
                case Msgs::eLoadMsgCode::ERROR_INVALID_MSG:
                    LOG_WARN(Logger::Nodes::Subscriber, "Received invalid message, possible transport layer error");
                    break;
                case Msgs::eLoadMsgCode::ERROR_MISMATCH:
                    LOG_ERROR(Logger::Nodes::Subscriber, "Mismatch between sender and receiver, dropping message");
                    break;
                case Msgs::eLoadMsgCode::ERROR_IMPLEMENTATION:
                    ASSERT_MSG("Message implementation is erroneous, expect undefined behavior");
            }

            return loadCode;
        }

      protected:
        SubscriberBase() = default;

        MsgT& getMsg(void)
        {
            return this->_msg;
        }

        void triggerCallback(const MsgT& msg_)
        {
            static_cast<ImplT*>(this)->_processCallback(msg_);
        }

        MsgT _msg;
    };

    /**
     * @brief Subscriber -> Links the reception of a RoverCan2::Msgs to a user
     * specified callback. SubscriberStandalone is meant for standalone
     * functions (static/C style/Non-Member) callback without dynamic
     * allocation.
     *
     * @tparam MsgT Object will subscribe to this message type
     * @tparam CallerT
     */
    template<typename MsgT, typename CallbackT>
    class SubscriberStandalone : public SubscriberBase<SubscriberStandalone<MsgT, CallbackT>, MsgT>
    {
        VALIDATE_BASE_TYPE(Msgs::MsgBaseT, MsgT);
        static_assert(std::is_invocable_r<void, CallbackT, const MsgT&>::value, "Callback must be of type: (void)(const MsgT&)");

        friend SubscriberBase<SubscriberStandalone<MsgT, CallbackT>, MsgT>;

      public:
        explicit SubscriberStandalone(CallbackT callback_):
            _callback(callback_)
        {
        }

      protected:
        void _processCallback(const MsgT& msg_)
        {
            this->_callback(msg_);
        }

      private:
        CallbackT* _callback;
    };

    /**
     * @brief Subscriber -> Links the reception of a RoverCan2::Msgs to a user
     * specified callback. SubscriberMember is meant for class member callback
     * function wihtout dynamic allocation
     *
     * @tparam MsgT Object will subscribe to this message type
     * @tparam CallerT
     */
    template<typename MsgT, typename CallerT>
    class SubscriberMember : public SubscriberBase<SubscriberMember<MsgT, CallerT>, MsgT>
    {
        VALIDATE_BASE_TYPE(Msgs::MsgBaseT, MsgT);
        friend SubscriberBase<SubscriberMember<MsgT, CallerT>, MsgT>;

        typedef void (CallerT::*MemberCallback_t)(const MsgT&);

      public:
        explicit SubscriberMember(CallerT& context_, MemberCallback_t callback_):
            _context(context_),
            _callback(callback_)
        {
        }

      protected:
        void _processCallback(const MsgT& msg_)
        {
            (_context.*_callback)(msg_);
        }

      private:
        CallerT& _context;
        MemberCallback_t _callback;
    };

    template<typename MsgT, typename CallerT>
    SubscriberMember(CallerT&, void (CallerT::*)(const MsgT&)) -> SubscriberMember<MsgT, CallerT>;
}  // namespace RoverCan2

#endif  // ROVER_CAN2_SUBSCRIBER_HPP
