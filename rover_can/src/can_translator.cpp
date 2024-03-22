// Next steps:
//   1. Add ros
//      1.1 Add status publisher

#include <chrono>
// Network socket stuff
#include <net/if.h>
#include <fcntl.h>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/can_device_status.hpp"

// RoverCanLib
#include "rover_can_lib/config.hpp"
#include "rover_can_lib/can_device.hpp"
#include "rover_can_lib/msgs/heartbeat.hpp"
#include "rover_can_lib/msgs/propulsion_motor.hpp"

#define LOGGER_NAME "CanMasterNode"

// Forward declarations
int createSocket(const char *canNetworkName_);

volatile sig_atomic_t shutdownFlag = 0;
void signal_handler(int signo);

RoverCanLib::Msgs::PropulsionMotor msg_0x101;

class CanMaster : public rclcpp::Node
{
public:
    CanMaster(int canSocket_) : Node("can_master")
    {
        _canSocket = canSocket_;

        _pub_CanStatus = this->create_publisher<rover_msgs::msg::CanDeviceStatus>("/rover/can/device_status", 1);

        // Add messages type to msgsMap
        _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::FRONTLEFT_MOTOR] = &msg_0x101;

        // Add devices to the deviceMap
        _deviceMap.emplace((size_t)RoverCanLib::Constant::eDeviceId::FRONTLEFT_MOTOR, CanDevice(0x101u, this, &CanMaster::CB_Can_PropulsionMotor, _pub_CanStatus));

        this->askStateCanDevices();
        // Create in last to make sure everything is init before it gets called
        _timerLoop = this->create_wall_timer(std::chrono::microseconds(10), std::bind(&CanMaster::mainLoop, this));

    }

private:
    void mainLoop()
    {
        // Send heartbeat at 4 Hz (see define)
        if (this->updateHeartbeat() != RoverCanLib::Constant::eInternalErrorCode::OK)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Error while sending Heartbeat, trying to reconnect to the can network...");
            rclcpp::shutdown();
        }

        // Checking watchdog
        if (chonoCanWatchdog.getTime() > RoverCanLib::Constant::WATCHDOG_TIMEOUT_MS)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Watchdog triggered, trying to reconnect to the can network...");
            rclcpp::shutdown();
        }

        // Empty rx queue
        if (readMsgFromCanSocket() != RoverCanLib::Constant::eInternalErrorCode::OK)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Error detected on CAN socket, trying to reconnect to the network...");
            rclcpp::shutdown();
        }
    }

    RoverCanLib::Constant::eInternalErrorCode updateHeartbeat()
    {
        if (_timerHeartbeat.isDone())
        {
            RoverCanLib::Msgs::Heartbeat msgHeartbeat;

            can_frame canFrame;
            canFrame.can_id = (canid_t)RoverCanLib::Constant::eDeviceId::MASTER_COMPUTER_UNIT;
            msgHeartbeat.getMsg((uint8_t)RoverCanLib::Msgs::Heartbeat::eMsgID::DONT_USE, &canFrame, rclcpp::get_logger(LOGGER_NAME));

            if (write(_canSocket, &canFrame, sizeof(canFrame)) != sizeof(canFrame))
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Error while writting");
                return RoverCanLib::Constant::eInternalErrorCode::ERROR;
            }
        }

        return RoverCanLib::Constant::eInternalErrorCode::OK;
    }

    // Read all msg in the rx queue and parse them accordingly. Also resets watchdog if msg is received
    RoverCanLib::Constant::eInternalErrorCode readMsgFromCanSocket()
    {
        for (EVER)
        {
            struct can_frame frame;
            ssize_t bytes_read;
            bytes_read = read(_canSocket, &frame, sizeof(struct can_frame));
            if (bytes_read == sizeof(can_frame))
            {
                chonoCanWatchdog.restart();

                // Check if the device exist in the hash map and parse can msg
                if (_deviceMap.find((size_t)frame.can_id) != _deviceMap.end())
                {
                    _deviceMap.find((size_t)frame.can_id)->second.parseMsg(&frame);
                }
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                "Received msg with device ID: 0x%.3x which isn't defined in deviceMap, dropping",
                                frame.can_id);
                }
                continue;
            }

            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                return RoverCanLib::Constant::eInternalErrorCode::OK;
                break;
            }
            if (bytes_read == -1 && errno != EAGAIN && errno != EWOULDBLOCK)
            {
                return RoverCanLib::Constant::eInternalErrorCode::ERROR;
                break;
            }
        }

        return RoverCanLib::Constant::eInternalErrorCode::OK;
    }

    void CB_Can_PropulsionMotor(uint16_t id_, const can_frame *frameMsg)
    {
        if (id_ != (size_t)RoverCanLib::Constant::eDeviceId::FRONTLEFT_MOTOR &&
            id_ != (size_t)RoverCanLib::Constant::eDeviceId::FRONTRIGHT_MOTOR &&
            id_ != (size_t)RoverCanLib::Constant::eDeviceId::REARLEFT_MOTOR &&
            id_ != (size_t)RoverCanLib::Constant::eDeviceId::REARRIGHT_MOTOR)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                         "Provided device ID: 0x%.3x isn't a propulsion motor",
                         id_);

            return;
        }

        if (frameMsg->data[(size_t)RoverCanLib::Constant::eDataIndex::MSG_ID] == (size_t)RoverCanLib::Constant::eMsgId::PROPULSION_MOTOR)
        {
#warning TODO: Implement data copy
            _msgsMap.find(id_)->second->parseMsg(frameMsg, rclcpp::get_logger(LOGGER_NAME));
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                         "Received unexpected msg id: 0x%.2x Possible mismatch in library version between nodes",
                         frameMsg->data[(size_t)RoverCanLib::Constant::eDataIndex::MSG_ID]);
        }
    }

    // Send a ErrorState msg to every devices registered in deviceMap.
    // They should all answer with their receptive state
    RoverCanLib::Constant::eInternalErrorCode askStateCanDevices()
    {
        RoverCanLib::Msgs::ErrorState msgErrorState;
        msgErrorState.data.error = 0;
        msgErrorState.data.warning = 0;

        return msgErrorState.sendMsg(RoverCanLib::Constant::eDeviceId::MASTER_COMPUTER_UNIT, _canSocket, rclcpp::get_logger(LOGGER_NAME));
    }

    int _canSocket;

    Timer<uint64_t, millis> _timerHeartbeat = Timer<uint64_t, millis>((uint64_t)(1000.0f / RoverCanLib::Constant::HEARTBEAT_FREQ));
    Chrono<uint64_t, millis> chonoCanWatchdog;

    rclcpp::TimerBase::SharedPtr _timerLoop;
    rclcpp::Publisher<rover_msgs::msg::CanDeviceStatus>::SharedPtr _pub_CanStatus;

    std::unordered_map<size_t, CanDevice> _deviceMap;
    std::unordered_map<size_t, RoverCanLib::Msgs::Msg *> _msgsMap;
};

int main(int argc, char *argv[])
{
    signal(SIGINT, signal_handler);

    while (!shutdownFlag)
    {
        rclcpp::init(argc, argv);
        int canSocket = createSocket("canRovus");
        assert(canSocket > 0);

        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Canbus ready, starting loop");
        rclcpp::spin(std::make_shared<CanMaster>(canSocket));

        close(canSocket);
        sleep(1);
    }

    return 0;
}

// Ctrl+C signal handler
void signal_handler(int signo)
{
    RCLCPP_INFO(rclcpp::get_logger("[SHUTDOWN]"), "Shutdown ask by user, exiting...");
    (void)signo;
    shutdownFlag = 1;
}

// Returns a socket on success or < 0 on error
int createSocket(const char *canNetworkName_)
{
    int canSocket;
    if ((canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        RCLCPP_FATAL(rclcpp::get_logger(LOGGER_NAME), "Error creating socket");
        return -1;
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = if_nametoindex(canNetworkName_);
    if (bind(canSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        RCLCPP_FATAL(rclcpp::get_logger(LOGGER_NAME), "Error bind to socket");
        close(canSocket);
        return -1;
    }

    // Set socket to non-blocking mode
    if (fcntl(canSocket, F_SETFL, O_NONBLOCK) < 0)
    {
        RCLCPP_FATAL(rclcpp::get_logger(LOGGER_NAME), "Error updating socket parameters");
        close(canSocket);
        return -1;
    }

    return canSocket;
}
