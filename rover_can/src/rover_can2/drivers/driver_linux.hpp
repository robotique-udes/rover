#ifndef DRIVER_LINUX_HPP
#define DRIVER_LINUX_HPP

// ROVER
#include "rover_can2/drivers/driver_base.hpp"
#include "rover_lib2/helpers/log.hpp"

// LINUX
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>

DEFINE_LOG_NODE(Driverlinux, Logger::eNodeState::ON);

namespace RoverCan2::Drivers
{
    class DriverLinux : public DriverBase<DriverLinux>
    {
      public:
        DriverLinux() = default;

        void __init(void) {}

      private:
        void installDriver(void)
        {
            _socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);

            if (_socket < 0)
            {
                LOG_ERROR(Logger::Nodes::DriverLinux, "Error opening socket: %s", strerror(errno));
                return;
            }
        }

        int _socket;
    };
}  // namespace RoverCan2::Drivers

#endif  // DRIVER_LINUX_HPP
