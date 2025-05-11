#ifndef DRIVER_LINUX_HPP
#define DRIVER_LINUX_HPP

#include "rover_can2/can_msg.hpp"
#include "rover_can2/constant.hpp"
#include "rover_can2/drivers/driver_base.hpp"

#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/circular_buffer.hpp"
#include "rover_lib2/helpers/watchdog.hpp"
#include "rover_lib2/helpers/time.hpp"

#include <string>
#include <thread>
#include <chrono>
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
        static constexpr std::chrono::milliseconds INTERFACE_STABILIZATION_TIME_MS{1500};
        static constexpr std::chrono::milliseconds EAGAIN_SLEEP_TIME_MS{1};
        static constexpr uint64_t RECV_WATCHDOG_TIMEOUT_MS
            = 2ULL * 1'000ULL / static_cast<uint64_t>(Constant::MASTER_HEARTBEAT_RATE_HZ);

        enum class eState : size_t
        {
            UNINSTALLED,
            RUNNING,
            TX_QUEUE_FULL
        };

      public:
        DriverLinux(const std::string& interfaceName_ = "canRovus"):
            _interfaceName(interfaceName_),
            _state(eState::UNINSTALLED),
            _recvWatchdog(RECV_WATCHDOG_TIMEOUT_MS)
        {
        }

        ~DriverLinux()
        {
            this->cleanupCanSocket();
        }

        // init() is useless in this case
        void __init(void) {}

        void __update(void)
        {
            if (!_recvWatchdog.isOk())
            {
                LOG_DEBUG(Logger::Nodes::DriverLinux,
                          "Receive watchdog timeout: no CAN message received within the expected interval");
            }

            switch (_state)
            {
                case eState::UNINSTALLED:
                    this->createCanSocket();
                    break;
                case eState::RUNNING:
                    [[fallthrough]];
                case eState::TX_QUEUE_FULL:
                    this->processNewMessage();
                    break;
            }
        }

        std::optional<CanMsg> _getMsg(void)
        {
            return _msgBuffer.getValue();
        }

        bool _sendMsg(const CanMsg& canMsg_)
        {
            if (_state < eState::RUNNING)
            {
                LOG_DEBUG(Logger::Nodes::DriverLinux, "Can't send msg, driver not initialized.");
                return false;
            }

            if (_socket_fd < 0)
            {
                LOG_DEBUG(Logger::Nodes::DriverLinux, "Invalid socket file descriptor: %d", _socket_fd);
                return false;
            }

            can_frame frame;
            frame.can_id = static_cast<canid_t>(canMsg_.getCanID());
            frame.len = canMsg_.dataLength;

            if (canMsg_.msgData.size() < frame.len)
            {
                LOG_DEBUG(Logger::Nodes::DriverLinux,
                          "Implementation error, can msg data buffer size (%zu) is smaller than declared length (%u)",
                          canMsg_.msgData.size(),
                          frame.len);
                return false;
            }

            if (frame.len > Constant::CAN_MAX_DATA_LENGTH)
            {
                LOG_DEBUG(Logger::Nodes::DriverLinux,
                          "Implementation error, can msg data size (%u) is bigger than max (%lu)",
                          frame.len,
                          Constant::CAN_MAX_DATA_LENGTH);

                return false;
            }
            else if (frame.len < TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA))
            {
                LOG_DEBUG(Logger::Nodes::DriverLinux,
                          "Implementation error, can msg data size (%u) is lower than min (%u)",
                          frame.len,
                          TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA));

                return false;
            }

            std::memcpy(frame.data, canMsg_.msgData.data(), frame.len);

            int bytesSent = write(_socket_fd, &frame, sizeof(struct can_frame));
            if (bytesSent == sizeof(struct can_frame))
            {
                LOG_DEBUG(Logger::Nodes::DriverLinux,
                          "Msg queued for transmission successfully, ID: %u, MsgID: %u, ContentID %u",
                          canMsg_.getCanID(),
                          canMsg_.getMsgID(),
                          canMsg_.getMsgContentID());

                _state = eState::RUNNING;  // if state is TX_QUEUE_FULL, return to RUNNING
                return true;
            }
            else if (bytesSent == -1)
            {
                switch (errno)
                {
                    case EINVAL:
                        LOG_DEBUG(Logger::Nodes::DriverLinux,
                                  "Invalid CAN frame arguments: errno=%d (%s)",
                                  errno,
                                  strerror(errno));
                        this->handleDeviceDisconnection();
                        break;
                    case ENOBUFS:
                        LOG_WARN(Logger::Nodes::DriverLinux, "TX buffer full: errno=%d (%s)", errno, strerror(errno));
                        _state = eState::TX_QUEUE_FULL;
                        break;
                    case ENXIO:
                        LOG_DEBUG(Logger::Nodes::DriverLinux,
                                  "Device not found (e.g., CAN adapter unplugged): errno=%d (%s)",
                                  errno,
                                  strerror(errno));
                        this->handleDeviceDisconnection();
                        break;
                    case EAGAIN:
                        LOG_DEBUG(Logger::Nodes::DriverLinux,
                                  "Non-blocking socket, no buffer space available: errno=%d (%s)",
                                  errno,
                                  strerror(errno));
                        break;
                    default:
                        LOG_DEBUG(Logger::Nodes::DriverLinux,
                                  "Unknown error sending CAN frame: errno=%d (%s)",
                                  errno,
                                  strerror(errno));
                        this->handleDeviceDisconnection();
                        break;
                }
            }
            else
            {
                // Partial frame write (should not happen on RAW CAN)
                LOG_DEBUG(Logger::Nodes::DriverLinux,
                          "Partial CAN frame sent: %d/%lu bytes",
                          bytesSent,
                          sizeof(struct can_frame));
            }
            return false;
        }

      private:
        bool createCanSocket(void)
        {
            _socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
            if (_socket_fd < 0)
            {
                LOG_DEBUG(Logger::Nodes::DriverLinux, "Failed to create CAN socket: errno=%d (%s)", errno, strerror(errno));
                return false;
            }

            struct ifreq ifr;
            std::strncpy(ifr.ifr_name, _interfaceName.c_str(), IFNAMSIZ - 1);
            if (ioctl(_socket_fd, SIOCGIFINDEX, &ifr) < 0)
            {
                LOG_DEBUG(Logger::Nodes::DriverLinux,
                          "Failed to get CAN interface index for '%s': errno=%d (%s)",
                          ifr.ifr_name,
                          errno,
                          strerror(errno));
                this->cleanupCanSocket();
                return false;
            }

            sockaddr_can addr;
            addr.can_family = PF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;

            if (bind(_socket_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
            {
                LOG_DEBUG(Logger::Nodes::DriverLinux,
                          "Failed to bind CAN socket to interface '%s': errno=%d (%s)",
                          ifr.ifr_name,
                          errno,
                          strerror(errno));
                this->cleanupCanSocket();
                return false;
            }

            int flags = fcntl(_socket_fd, F_GETFL, 0);
            if (flags < 0 || fcntl(_socket_fd, F_SETFL, flags | O_NONBLOCK) < 0)
            {
                LOG_DEBUG(Logger::Nodes::DriverLinux,
                          "Failed to set CAN socket to non-blocking mode: errno=%d (%s)",
                          errno,
                          strerror(errno));
                cleanupCanSocket();
                return false;
            }

            // Ugly for now but give time for the CAN interface to stabilize (e.g., after USB-CAN device insertion)
            std::this_thread::sleep_for(INTERFACE_STABILIZATION_TIME_MS);

            LOG_INFO(Logger::Nodes::DriverLinux, "CAN socket successfully created");
            _state = eState::RUNNING;
            return true;
        }

        void cleanupCanSocket(void)
        {
            if (_socket_fd >= 0)
            {
                close(_socket_fd);
                _socket_fd = -1;
            }
        }

        void processNewMessage(void)
        {
            struct can_frame frame;
            ssize_t nbytes = read(_socket_fd, &frame, sizeof(frame));

            if (nbytes == -1)
            {
                switch (errno)
                {
                    case EAGAIN:
                        // Non-blocking read: no data available now
                        std::this_thread::sleep_for(EAGAIN_SLEEP_TIME_MS);
                        _recvWatchdog.reset();
                        return;
                    case EIO:
                        LOG_WARN(Logger::Nodes::DriverLinux,
                                 "CAN I/O error, possible hardware or driver fault : errno=%d (%s)",
                                 errno,
                                 strerror(errno));
                        this->handleDeviceDisconnection();
                        return;
                    default:
                        LOG_DEBUG(Logger::Nodes::DriverLinux, "Error receiving CAN frame: errno=%d (%s)", errno, strerror(errno));
                        return;
                }
            }
            _recvWatchdog.reset();

            RoverCan2::CanMsg msg
                = RoverCan2::CanMsg(static_cast<RoverCan2::Constant::eDeviceId>(frame.can_id), frame.data, frame.len);

            if (msg.getMsgID() == RoverCan2::Constant::eMsgId::INVALID)
            {
                LOG_WARN(Logger::Nodes::DriverLinux, "Received msg with invalid ID: %u dropping", TO_UNDERLYING(msg.getMsgID()));
                return;
            }

            CircularBufferT::eErrorCode status = _msgBuffer.addValue(msg);
            switch (status)
            {
                case decltype(_msgBuffer)::eErrorCode::SUCCESS:
                    break;
                case decltype(_msgBuffer)::eErrorCode::SUCCESS_DATA_LOSS:
                    LOG_WARN(Logger::Nodes::DriverLinux, "Msg buffer full, losing data");
                    break;
                case decltype(_msgBuffer)::eErrorCode::ERROR:
                    LOG_WARN(Logger::Nodes::DriverLinux, "Unknown error");
                    break;
            }

            LOG_DEBUG(Logger::Nodes::DriverLinux, "Received CAN ID: %u, Length: %u", frame.can_id, frame.len);
        }

        void handleDeviceDisconnection(void)
        {
            LOG_WARN(Logger::Nodes::DriverLinux, "USB to CAN device likely unplugged, trying to reconnect ..");
            cleanupCanSocket();
            _state = eState::UNINSTALLED;
        }

        int _socket_fd = -1;

        std::string _interfaceName;
        eState _state;
        CircularBuffer<CanMsg, 10UL> _msgBuffer;
        Watchdog<uint64_t, Time::millis> _recvWatchdog;
    };
}  // namespace RoverCan2::Drivers

#endif  // DRIVER_LINUX_HPP