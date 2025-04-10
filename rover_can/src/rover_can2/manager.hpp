#ifndef MANAGER_HPP
#define MANAGER_HPP

#include "rover_can2/msgs/error_state.hpp"

#include "rover_can2/drivers/driver_base.hpp"
#include "rover_can2/device.hpp"
#include "rover_can2/constant.hpp"

#include "rover_lib2/rover_object.hpp"
#include "rover_lib2/helpers/health_state.hpp"

#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_lib2/helpers/time.hpp"

#include <tuple>
#include <optional>

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

    template<typename DriverT, typename... DevicesT>
    class Manager : public RoverObject<Manager<DriverT, DevicesT...>>,
                    ManagerT
    {
        VALIDATE_BASE_TYPE(Drivers::DriverBaseT, DriverT);
        VALIDATE_BASE_TYPE_PACK(DeviceT, DevicesT);

        static constexpr uint8_t MAX_MSG_PARSE_PER_UPDATE = 10U;
        static constexpr Constant::eDeviceId ERROR_STATE_HANDLER_ID = Constant::eDeviceId::MASTER_COMPUTER_UNIT;
        static constexpr unsigned long ERROR_STATE_REPORTING_PERIOD_S = 2'000UL;

      public:
        Manager(DriverT& driver_, DevicesT&&... devices_):
            _driver(driver_),
            _canDevices(std::forward<DevicesT>(devices_)...),
            _dev_Master(ERROR_STATE_HANDLER_ID,
                        SubscriberMember<Msgs::ErrorState, Manager<DriverT, DevicesT...>>{
                            *this,
                            &Manager<DriverT, DevicesT...>::CB_ErrorStateFromMaster}),
            _errorStateReportingLoop(ERROR_STATE_REPORTING_PERIOD_S)
        {
        }

        void _init(void)
        {
            _driver.init();
        }

        void _update(void)
        {
            _driver.update();

            if (_errorStateReportingLoop.isReady() && HealthState::getInstance().getInError())
            {
                this->reportErrorStateToMaster();
            }

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
        bool parseMsgAllDevices(const CanMsg& msg_)
        {
            bool allSuccess = true;

            std::apply(
                [&](DevicesT&&... devices_)
                {
                    ((allSuccess &= parseMsgSingleDevices(devices_, msg_)), ...);
                },
                _canDevices);

            allSuccess &= parseMsgSingleDevices(_dev_Master, msg_);

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

        void CB_ErrorStateFromMaster(const Msgs::ErrorState& /*msg_*/)
        {
            this->reportErrorStateToMaster();
        }

        void reportErrorStateToMaster(void)
        {
            std::apply(
                [&](DevicesT&... device)
                {
                    ((this->sendDeviceErrorState(device)), ...);
                },
                _canDevices);
        }

        template<typename DeviceT>
        void sendDeviceErrorState(DeviceT& device_)
        {
            static_assert(std::is_base_of_v<DeviceT, DeviceT>, "Template parameter must be of type CanDeviceT");

            Msgs::ErrorState msg;
            msg.data().error = HealthState::getInstance().getInError();

            if (!this->sendMsg(device_.getCanId(), msg, true))
            {
                LOG_WARN(Logger::Nodes::CanManager,
                         "Unable to send error state msg for device id: %u",
                         TO_UNDERLYING(device_.getCanId()));
            }
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
        Device<SubscriberMember<Msgs::ErrorState, Manager<DriverT, DevicesT...>>> _dev_Master;
        LoopTimer<uint64_t, Time::millis> _errorStateReportingLoop;
    };

    template<typename DriverT, typename... DevicesT>
    Manager(DriverT canDriver_, DevicesT&&...) -> Manager<DriverT, DevicesT...>;
}  // namespace RoverCan2

#endif  // MANAGER_HPP
