#ifndef ROVER_CAN2_DEVICE_HPP
#define ROVER_CAN2_DEVICE_HPP

#include "rover_can2/constant.hpp"
#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/publisher.hpp"
#include "rover_can2/subscriber.hpp"

#include <tuple>
#include <type_traits>

namespace RoverCan2
{
    /**
     * @brief Base Device class. Allows template abstraction for type validation
     *
     */
    class DeviceT
    {
      public:
        enum class eReturnValue
        {
            NOT_CONCERNED,
            SUCCESS,
            FAILED,
        };

      protected:
        DeviceT() = default;
    };

    /**
     * @brief Class which links multiples pubs and subs to ultimatly be a Device on the canbus network. Users should extends
     * this class to creates custom device the same way a rclcpp::node works in ROS2.
     *
     * @tparam Pubs_SubsT
     */
    template<typename... Pubs_SubsT>
    class Device : public DeviceT
    {
        // clang-format off
        static_assert((... && (std::is_base_of_v<SubscriberBaseT, std::remove_reference_t<Pubs_SubsT>> 
                              || std::is_base_of_v<PublisherBaseT, std::remove_reference_t<Pubs_SubsT>>)),
                      "All template arguments must be derived from SubscriberBaseT or PublisherBaseT");
        // clang-format on

        static constexpr size_t MAX_MSG_PARSED_PER_LOOP = 10U;

      public:
        explicit Device(RoverCan2::Constant::eDeviceId id_, Pubs_SubsT&&... subs_):
            _id(id_),
            _pubs_subs(std::forward<Pubs_SubsT>(subs_)...)
        {
        }

        /**
         * @brief Tries to load a CanMsg into matching subscribers
         * @attention [WARNING] Internal use only
         */
        bool parseMsg(const CanMsg& msgCan_)
        {
            bool success = true;
            if (msgCan_.getCanID() != this->getCanId())
            {
                // Not concerned but no error either
                return success;
            }

            std::apply(
                [&](Pubs_SubsT&... sub_)
                {
                    ((success &= this->loadMsgSub(sub_, msgCan_)), ...);
                },
                _pubs_subs);

            return success;
        }

        /**
         * @brief Queues the passed msg into a matching publisher's send buffer. The msg will be sent during the next call
         * to sendPubsQueuedMsgs().
         * @attention It's suggested to let the Manager call the sendPubsQueuedMsgs in its update() loop instead of doing it
         * manually. The manager needs to be passed to the function anyway.
         */
        template<typename MsgT>
        eReturnValue sendMsg(const MsgT& msg_)
        {
            VALIDATE_BASE_TYPE(Msgs::MsgBaseT, MsgT);

            if constexpr (sizeof...(Pubs_SubsT) == 0)
            {
                return eReturnValue::NOT_CONCERNED;
            }
            else
            {
                eReturnValue retval = eReturnValue::NOT_CONCERNED;
                auto handleFuncResult = [&](eReturnValue newResult_)
                {
                    if (TO_UNDERLYING(newResult_) > TO_UNDERLYING(retval))
                    {
                        retval = newResult_;
                    }
                };

                std::apply(
                    [&](Pubs_SubsT&... pub_sub_)
                    {
                        (handleFuncResult(this->sendPubMsg(pub_sub_, msg_)), ...);
                    },
                    _pubs_subs);

                return retval;
            }
        }

        /**
         * @brief Sends all the device's publishers queued msgs.
         * @attention It's suggested to let the Manager call this method in it's update() loop instead of doing it
         * manually.
         * @attention [WARNING] Internal use only
         */
        template<typename ManagerT>
        bool sendPubsQueuedMsgs(ManagerT& manager_)
        {
            VALIDATE_BASE_TYPE(ManagerT, ManagerT);

            bool allSuccess = true;
            std::apply(
                [&](Pubs_SubsT&... pub_)
                {
                    ((allSuccess &= this->sendPubQueuedMsg(manager_, pub_)), ...);
                },
                _pubs_subs);

            return allSuccess;
        }

        /**
         * @brief Return the current device CAN ID
         *
         * @return constexpr RoverCan2::Constant::eDeviceId
         */
        constexpr RoverCan2::Constant::eDeviceId getCanId(void) const
        {
            return _id;
        }

      private:
        template<typename SubT>
        bool loadMsgSub(SubT& sub_, const CanMsg& msgCan_)
        {
            if constexpr (std::is_base_of_v<SubscriberBaseT, std::remove_reference_t<decltype(sub_)>>)
            {
                bool success = true;
                Msgs::eLoadMsgCode loadCode = sub_.parseMsg(msgCan_);
                switch (loadCode)
                {
                    case Msgs::eLoadMsgCode::SUCCESS_COMPLETE:
                        break;
                    case Msgs::eLoadMsgCode::SUCCESS_INCOMPLETE:
                        break;
                    case Msgs::eLoadMsgCode::NOT_CONCERNED:
                        break;
                    case Msgs::eLoadMsgCode::ERROR_INVALID_MSG:
                        [[fallthrough]];
                    case Msgs::eLoadMsgCode::ERROR_MISMATCH:
                        [[fallthrough]];
                    case Msgs::eLoadMsgCode::ERROR_IMPLEMENTATION:
                        [[fallthrough]];
                    default:
                        success = false;
                        break;
                }

                return success;
            }

            return true;  // Not concerned
        }

        template<typename ManagerT, typename PubT>
        bool sendPubQueuedMsg(ManagerT& manager_, PubT& pub_)
        {
            VALIDATE_BASE_TYPE(ManagerT, ManagerT);

            if constexpr (std::is_base_of_v<PublisherBaseT, std::remove_reference_t<decltype(pub_)>>)
            {
                return pub_.sendQueuedMsgs(this->getCanId(), manager_);
            }

            return true;  // Not concerned
        }

        template<typename PubSubT, typename MsgT>
        eReturnValue sendPubMsg(PubSubT& pub_, const MsgT& msg_)
        {
            VALIDATE_BASE_TYPE(Msgs::MsgBaseT, MsgT);

            if constexpr (std::is_base_of_v<PublisherBase<MsgT>, std::remove_reference_t<decltype(pub_)>>)
            {
                CircularBufferT::eErrorCode retVal = pub_.queueMsg(msg_);
                if (retVal == CircularBufferT::eErrorCode::SUCCESS || retVal == CircularBufferT::eErrorCode::SUCCESS_DATA_LOSS)
                {
                    return eReturnValue::SUCCESS;
                }
                else
                {
                    return eReturnValue::FAILED;
                }
            }

            return eReturnValue::NOT_CONCERNED;
        }

        const RoverCan2::Constant::eDeviceId _id;
        std::tuple<Pubs_SubsT...> _pubs_subs;
    };

    template<typename... Pubs_SubsT>
    Device(RoverCan2::Constant::eDeviceId id_, Pubs_SubsT&&...) -> Device<Pubs_SubsT...>;

}  // namespace RoverCan2

#endif  // ROVER_CAN2_DEVICE_HPP
