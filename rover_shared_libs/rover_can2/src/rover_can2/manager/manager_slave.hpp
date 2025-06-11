#ifndef MANAGER_SLAVE_HPP
#define MANAGER_SLAVE_HPP

#include "rover_can2/msgs/error_state.hpp"

#include "rover_can2/manager/manager.hpp"

#include <rover_lib2/rover_object.hpp>
#include <rover_lib2/helpers/health_state.hpp>
#include <rover_lib2/helpers/loop_timer.hpp>

DEFINE_LOG_NODE(CanManagerSlave, Logger::eNodeState::OFF);

namespace RoverCan2
{
    template<typename DriverT, typename... DevicesT>
    class ManagerSlave : public Manager<ManagerSlave<DriverT, DevicesT...>, DriverT, DevicesT...>
    {
        using BaseT = Manager<ManagerSlave<DriverT, DevicesT...>, DriverT, DevicesT...>;
        friend BaseT;

        static constexpr Constant::eDeviceId ERROR_STATE_HANDLER_ID = Constant::eDeviceId::MASTER_COMPUTER_UNIT;
        static constexpr unsigned long ERROR_STATE_REPORTING_PERIOD_S = 2'000UL;

      public:
        ManagerSlave(DriverT& driver_, DevicesT&&... devices_):
            BaseT(driver_, std::forward<DevicesT>(devices_)...),
            _dev_Master(ERROR_STATE_HANDLER_ID,
                        SubscriberMember<Msgs::ErrorState, ManagerSlave<DriverT, DevicesT...>>{
                            *this,
                            &ManagerSlave<DriverT, DevicesT...>::CB_ErrorStateFromMaster}),
            _errorStateReportingLoop(ERROR_STATE_REPORTING_PERIOD_S)
        {
        }

        void __update(void)
        {
            if (HealthState::getInstance().getInError() && _errorStateReportingLoop.isReady())
            {
                this->reportErrorStateToMaster();
            }
        }

      private:
        bool _parseMsgAllDevices(const CanMsg& msg_)
        {
            return this->parseMsgSingleDevices(_dev_Master, msg_);
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
                this->_canDevices);
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

        Device<SubscriberMember<Msgs::ErrorState, ManagerSlave<DriverT, DevicesT...>>> _dev_Master;
        LoopTimer<uint64_t, Time::millis> _errorStateReportingLoop;
    };

    template<typename DriverT, typename... DevicesT>
    ManagerSlave(DriverT canDriver_, DevicesT&&...) -> ManagerSlave<DriverT, DevicesT...>;

}  // namespace RoverCan2

#endif  // MANAGER_SLAVE_HPP
