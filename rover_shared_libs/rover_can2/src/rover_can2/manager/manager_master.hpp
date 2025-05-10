#ifndef MANAGER_MASTER_HPP
#define MANAGER_MASTER_HPP

#include "rover_can2/constant.hpp"
#include "rover_can2/device.hpp"
#include "rover_can2/msgs/error_state.hpp"

#include "rover_can2/manager/manager.hpp"
#include "rover_can2/publisher.hpp"
#include "rover_lib2/helpers/log.hpp"

#include <functional>
#include <rover_lib2/rover_object.hpp>
#include <rover_lib2/helpers/health_state.hpp>
#include <rover_lib2/helpers/loop_timer.hpp>

DEFINE_LOG_NODE(CanManagerMaster, Logger::eNodeState::OFF);

namespace RoverCan2
{
    template<typename DriverT, typename... DevicesT>
    class ManagerMaster : public Manager<ManagerMaster<DriverT, DevicesT...>, DriverT, DevicesT...>
    {
        using Implt = ManagerMaster<DriverT, DevicesT...>;
        using BaseT = Manager<Implt, DriverT, DevicesT...>;
        friend BaseT;

        static constexpr RoverCan2::Constant::eDeviceId ERROR_STATE_HANDLER_ID
            = RoverCan2::Constant::eDeviceId::MASTER_COMPUTER_UNIT;

      public:
        ManagerMaster(DriverT& driver_,
                      std::function<void(Constant::eDeviceId, const Msgs::ErrorState&)> callbackOnErrorStateRecv_,
                      DevicesT&&... devices_):
            BaseT(driver_, std::forward<DevicesT>(devices_)...),
            callbackOnErrorStateRecv(std::move(callbackOnErrorStateRecv_))
        {
        }

        void __update(void) {}

        bool sendErrorStateRequest(void)
        {
            /* A device receiving a error state message from master should report it's current Health/ErrorState (see ManagerSlave
             * class for implementation) */

            RoverCan2::Msgs::ErrorState msg;
            return this->sendMsg(ERROR_STATE_HANDLER_ID, msg, true);
        }

      private:
        bool _parseMsgAllDevices(const CanMsg& msg_)
        {
            if (msg_.getMsgID() == Constant::eMsgId::ERROR_STATE)
            {
                if (msg_.getCanID() != Constant::eDeviceId::MASTER_COMPUTER_UNIT)
                {
                    LOG_WARN(Logger::Nodes::CanManagerMaster,
                             "Received message from another Master, having multiple Can Master will cause undefined behavior");
                    return false;
                }

                this->handleErrorStateReception(msg_);
            }

            return true;
        }

        void handleErrorStateReception(const CanMsg& canMsg_)
        {
            Msgs::ErrorState msg;
            msg.loadMsg(canMsg_);

            this->callbackOnErrorStateRecv(canMsg_.getCanID(), msg);
        }

        std::function<void(Constant::eDeviceId, const Msgs::ErrorState&)> callbackOnErrorStateRecv;
    };

    template<typename DriverT, typename... DevicesT>
    ManagerMaster(DriverT&, std::function<void(Constant::eDeviceId, const Msgs::ErrorState&)>, DevicesT&&...)
        -> ManagerMaster<DriverT, DevicesT...>;
}  // namespace RoverCan2

#endif  // MANAGER_MASTER_HPP
