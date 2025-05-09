#ifndef DRIVER_LINUX_HPP
#define DRIVER_LINUX_HPP

#include "rover_can2/constant.hpp"
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

/*
TODO

- Handle deconnection and reconnection of USB to CAN device.
- Send msg
- Receive msg
- get msg in circular buffer

*/

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

        bool _sendMsg(const CanMsg& canMsg_)
        {
            if (_socket_fd < 0)
            {
                LOG_ERROR(Logger::Nodes::DriverLinux, "Invalid socket file descriptor: %d", _socket_fd);
                return false;
            }

            if (_state != eState::RUNNING)
            {
                LOG_WARN(Logger::Nodes::DriverLinux,
                         "Can't send msg, driver is not in a valid state to send messages. Expected state >= %u but current "
                         "state is: %u. Msg dropped",
                         TO_UNDERLYING(eState::RUNNING),
                         TO_UNDERLYING(_state));
                return false;
            }

            struct can_frame frame{};
            frame.can_id = static_cast<uint32_t>(canMsg_.getCanID());
            frame.can_dlc = canMsg_.dataLength;

            if (frame.can_dlc > Constant::CAN_MAX_DATA_LENGTH)
            {
                LOG_ERROR(Logger::Nodes::DriverLinux,
                          "Implementation error, can msg data size (%u) is bigger than max (%u)",
                          frame.can_dlc,
                          Constant::CAN_MAX_DATA_LENGTH);

                return false;
            }
            else if (frame.can_dlc < TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA))
            {
                LOG_ERROR(Logger::Nodes::DriverLinux,
                          "Implementation error, can msg data size (%u) is lower than min (%u)",
                          frame.can_dlc,
                          TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA));

                return false;
            }

            std::memcpy(frame.data, canMsg_.msgData.data(), frame.can_dlc);

            int bytes_sent = write(_socket_fd, &frame, sizeof(struct can_frame));
            if (bytes_sent == sizeof(struct can_frame))
            {
                LOG_INFO(Logger::Nodes::DriverLinux,
                         "Msg queued for transmission successfully, ID: %u, MsgID: %u, ContentID %u",
                         canMsg_.getCanID(),
                         canMsg_.getMsgID(),
                         canMsg_.getMsgContentID());
                return true;
            }
            else if (bytes_sent == -1)
            {
                switch (errno)
                {
                    case EINVAL:
                        LOG_ERROR(Logger::Nodes::DriverLinux,
                                  "Invalid CAN frame arguments: errno=%d (%s)",
                                  errno,
                                  strerror(errno));
                        break;
                    case ENOBUFS:
                        LOG_WARN(Logger::Nodes::DriverLinux, "TX buffer full: errno=%d (%s)", errno, strerror(errno));
                        break;
                    case ENXIO:
                        LOG_ERROR(Logger::Nodes::DriverLinux,
                                  "Device not found (e.g., CAN adapter unplugged): errno=%d (%s)",
                                  errno,
                                  strerror(errno));
                        break;
                    case EAGAIN:
                        LOG_WARN(Logger::Nodes::DriverLinux,
                                 "Non-blocking socket, no buffer space available: errno=%d (%s)",
                                 errno,
                                 strerror(errno));
                        break;
                    default:
                        LOG_ERROR(Logger::Nodes::DriverLinux,
                                  "Unknown error sending CAN frame: errno=%d (%s)",
                                  errno,
                                  strerror(errno));
                        break;
                }
            }
            else
            {
                // Partial frame write (should not happen on RAW CAN)
                LOG_ERROR(Logger::Nodes::DriverLinux,
                          "Partial CAN frame sent: %d/%lu bytes",
                          bytes_sent,
                          sizeof(struct can_frame));
            }
            return false;
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

            struct sockaddr_can addr{};
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
