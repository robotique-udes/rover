#ifndef DRIVER_LINUX_HPP
#define DRIVER_LINUX_HPP

#include "rover_can2/drivers/driver_base.hpp"
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/circular_buffer.hpp"
#include "rover_lib2/helpers/watchdog.hpp"
#include "rover_lib2/helpers/time.hpp"

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
        enum eState : size_t
        {
            UNINSTALLED,
            RUNNING,
            INVALID_STATE,
        };

      public:
        DriverLinux():
            _recvWatchdog(2ULL * 1'000ULL / static_cast<uint64_t>(Constant::MASTER_HEARTBEAT_RATE_HZ))
        {
        }

        void __init()
        {
            LOG_INFO(Logger::Nodes::DriverLinux, "Initializing CAN linux driver");

            if (createCanSocket())
            {
                _state = eState::RUNNING;
            }
            else
            {
                _state = eState::UNINSTALLED;
            }
        }

        void __update(void)
        {
            RoverCan2::CanMsg outMsg;
            receiveMsg(outMsg);
        }

        bool _sendMsg(const CanMsg& msg_)
        {
            struct can_frame frame
            {
            };
            frame.can_id = static_cast<uint32_t>(msg_.getCanID());
            frame.can_dlc = msg_.dataLength;

            std::memcpy(frame.data, msg_.msgData.data(), frame.can_dlc);

            int bytes_sent = write(_socket_fd, &frame, sizeof(struct can_frame));
            if (bytes_sent != sizeof(struct can_frame))
            {
                LOG_ERROR(Logger::Nodes::DriverLinux, "Failed to send CAN message");
                return false;
            }

            LOG_INFO(Logger::Nodes::DriverLinux, "Sent CAN message with ID: %i", frame.can_id);
            return true;
        }

        bool receiveMsg(RoverCan2::CanMsg& outMsg)
        {
            int flags = fcntl(_socket_fd, F_GETFL, 0);
            fcntl(_socket_fd, F_SETFL, flags | O_NONBLOCK);

            struct can_frame frame;
            ssize_t nbytes = read(_socket_fd, &frame, sizeof(frame));

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

        bool createCanSocket()
        {
            if ((_socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
            {
                LOG_ERROR(Logger::Nodes::DriverLinux, "Impossible to create can socket !");
                return false;
            }

            struct ifreq ifr;
            std::strncpy(ifr.ifr_name, "canRovus", IFNAMSIZ);
            if (ioctl(_socket_fd, SIOCGIFINDEX, &ifr) < 0)
            {
                LOG_ERROR(Logger::Nodes::DriverLinux, "Failed to get interface index");
                return false;
            }

            struct sockaddr_can addr
            {
            };
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;

            if (bind(_socket_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
            {
                LOG_ERROR(Logger::Nodes::DriverLinux, "Failed to bind CAN socket");
                return false;
            }

            return true;
        }

      private:
        int _socket_fd = -1;

        eState _state;

        CircularBuffer<CanMsg, 10UL> _msgBuffer;
        Watchdog<uint64_t, Time::millis> _recvWatchdog;
    };
}  // namespace RoverCan2::Drivers

#endif  // DRIVER_LINUX_HPP
