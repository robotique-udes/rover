#ifndef DRIVER_LINUX_HPP
#define DRIVER_LINUX_HPP

#include "rover_can2/drivers/driver_base.hpp"
#include "rover_lib2/helpers/log.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>

DEFINE_LOG_NODE(DriverLinux, Logger::eNodeState::ON);

namespace RoverCan2::Drivers
{
    class DriverLinux : public DriverBase<DriverLinux>
    {
      public:
        DriverLinux() = default;

        void __init()
        {
            LOG_INFO(Logger::Nodes::DriverLinux, "Initializing CAN linux driver");

            if ((socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
            {
                LOG_ERROR(Logger::Nodes::DriverLinux, "Impossible to create can socket !");
            }

            struct ifreq ifr;
            std::strncpy(ifr.ifr_name, "canRovus", IFNAMSIZ);
            if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0)
            {
                LOG_ERROR(Logger::Nodes::DriverLinux, "Failed to get interface index");
                return;
            }

            struct sockaddr_can addr{};
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;

            if (bind(socket_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
            {
                throw std::runtime_error("Failed to bind CAN socket");
            }
        }

        bool _sendMsg(const CanMsg& msg_)
        {
            struct can_frame frame{};
            frame.can_id = static_cast<uint32_t>(msg_.getCanID());
            frame.can_dlc = msg_.dataLength;

            std::memcpy(frame.data, msg_.msgData.data(), frame.can_dlc);

            int bytes_sent = write(socket_fd, &frame, sizeof(struct can_frame));
            if (bytes_sent != sizeof(struct can_frame))
            {
                LOG_ERROR(Logger::Nodes::DriverLinux, "Failed to send CAN message");
                return false;
            }

            LOG_INFO(Logger::Nodes::DriverLinux, "Sent CAN message with ID: " + std::to_string(frame.can_id));
            return true;
        }

        std::optional<CanMsg> getMsg(void)
        {
            return static_cast<Impl_T*>(this)->_getMsg();
        }

      private:
        int socket_fd = -1;
    };
}  // namespace RoverCan2::Drivers

#endif  // DRIVER_LINUX_HPP
