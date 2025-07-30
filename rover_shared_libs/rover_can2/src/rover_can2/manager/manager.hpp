#ifndef ROVER_CAN2_MANAGER_MANAGER_HPP
#define ROVER_CAN2_MANAGER_MANAGER_HPP

#include "rover_can2/drivers/driver_base.hpp"
#include "rover_can2/device.hpp"

DEFINE_LOG_NODE(CanManager, Logger::eNodeState::OFF);

namespace RoverCan2
{
    /**
     * @brief Base Manager class. Allows template abstraction for type validation
     */
    class ManagerT
    {
      protected:
        ManagerT() = default;
    };

    template<typename ImplT, typename DriverT, typename... DevicesT>
    class Manager : public ManagerT
    {
        VALIDATE_BASE_TYPE(Drivers::DriverBaseT, DriverT);
        VALIDATE_BASE_TYPE_PACK(DeviceT, DevicesT);
        friend ImplT;

        static constexpr uint8_t MAX_MSG_PARSE_PER_UPDATE = 10U;

      public:
        void init(void)
        {
            _driver.init();
        }

        void update(void)
        {
            _driver.update();

            std::optional<CanMsg> msgOpt = std::nullopt;
            for (uint8_t i = 0U; i < MAX_MSG_PARSE_PER_UPDATE; i++)
            {
                msgOpt = _driver.getMsg();
                if (!msgOpt.has_value())
                {
                    break;
                }

                this->parseMsgAllDevices(msgOpt.value());
            }

            this->publishAllQueuedMsgs();

            static_cast<ImplT*>(this)->_update();
        }

        /**
         * @brief Sends a message on the CAN network. Unless "sendEvenIfDeviceIdInvalid_" is set, a DeviceT must be "registered"
         * with the passed "senderId_" for the call to succeed.
         * @tparam MsgT Type of Msg which will be sent
         * @param senderId_
         * @param msg_
         * @param sendEvenIfDeviceIdInvalid_
         */
        template<typename MsgT>
        bool sendMsg(Constant::eDeviceId senderId_, const MsgT& msg_, bool sendEvenIfDeviceIdInvalid_ = false)
        {
            static_assert(std::is_base_of_v<Msgs::Msg<MsgT>, MsgT>,
                          "MsgT template argument must be CRTP child type of Msgs::Msg");

            if (!sendEvenIfDeviceIdInvalid_)
            {
                bool senderIdValid = false;
                std::apply(
                    [&](DevicesT&&... devices_)
                    {
                        ((senderIdValid |= (senderId_ == devices_.getCanId())), ...);
                    },
                    _canDevices);

                if (!senderIdValid)
                {
                    LOG_WARN(
                        Logger::Nodes::CanManager,
                        "Trying to send a can msg from device ID: %u, but this manager doesn't contain a device with this ID",
                        TO_UNDERLYING(senderId_));
                    return false;
                }
            }

            bool success = true;
            uint8_t nbOfMsgToSend = msg_.getMsgContentCount();
            for (uint8_t i = 0U; i < nbOfMsgToSend; i++)
            {
                std::optional<CanMsg> canMsgOpt = msg_.getCanMsg(i);
                if (!canMsgOpt)
                {
                    success = false;
                    continue;
                }

                canMsgOpt.value().setCanID(senderId_);
                RoverCan2::CanMsg canMsg = canMsgOpt.value();
                success = std::min(success, _driver.sendMsg(canMsg));
            }

            return success;
        }

      private:
        Manager(DriverT& driver_, DevicesT&&... devices_):
            _driver(driver_),
            _canDevices(std::forward<DevicesT>(devices_)...)
        {
        }

        bool parseMsgAllDevices(const CanMsg& msg_)
        {
            bool allSuccess = true;

            std::apply(
                [&](DevicesT&&... devices_)
                {
                    ((allSuccess &= parseMsgSingleDevices(devices_, msg_)), ...);
                },
                _canDevices);

            allSuccess &= static_cast<ImplT*>(this)->_parseMsgAllDevices(msg_);

            return allSuccess;
        }

        template<typename deviceT>
        bool parseMsgSingleDevices(deviceT& device_, const CanMsg& msg_)
        {
            bool success = false;
            success = device_.parseMsg(msg_);
            if (!success)
            {
                LOG_WARN(Logger::Nodes::CanManager,
                         "Error from CanDevice of ID: %u, while load message of type: %u",
                         device_.getCanId(),
                         TO_UNDERLYING(msg_.getMsgID()));
            }
            return success;
        }

        void publishAllQueuedMsgs(void)
        {
            std::apply(
                [&](DevicesT&... devices_)
                {
                    ((devices_.sendPubsQueuedMsgs(*this)), ...);
                },
                _canDevices);
        }

        DriverT& _driver;
        std::tuple<DevicesT...> _canDevices;
    };

}  // namespace RoverCan2

#endif  // ROVER_CAN2_MANAGER_MANAGER_HPP
