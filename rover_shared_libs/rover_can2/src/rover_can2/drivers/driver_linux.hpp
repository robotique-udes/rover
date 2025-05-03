#ifndef DRIVER_LINUX_HPP
#define DRIVER_LINUX_HPP

#include "rover_can2/drivers/driver_base.hpp"
#include "rover_lib2/helpers/log.hpp"

#include <fcntl.h>
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

        void __update(void)
        {
            RoverCan2::CanMsg outMsg;
            receiveMsg(outMsg);
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

        bool receiveMsg(RoverCan2::CanMsg& outMsg)
        {
            int flags = fcntl(socket_fd, F_GETFL, 0);
            fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK);

            struct can_frame frame;
            ssize_t nbytes = read(socket_fd, &frame, sizeof(frame));

            if (nbytes < 0)
            {
                LOG_ERROR(Logger::Nodes::DriverLinux, "CAN read error");
                return false;
            }
            else if (static_cast<size_t>(nbytes) < sizeof(struct can_frame))
            {
                LOG_ERROR(Logger::Nodes::DriverLinux, "Incomplete CAN frame");
                return false;
            }

            // Construct a RoverCan2::CanMsg from the raw frame
            outMsg = RoverCan2::CanMsg(static_cast<RoverCan2::Constant::eDeviceId>(frame.can_id), frame.data, frame.can_dlc);

            _sendMsg(outMsg);

            LOG_INFO(Logger::Nodes::DriverLinux, "Received CAN ID: %u, Length: %u", frame.can_id, frame.can_dlc);
            return true;
        }

      private:
        int socket_fd = -1;
    };
}  // namespace RoverCan2::Drivers

#endif  // DRIVER_LINUX_HPP
