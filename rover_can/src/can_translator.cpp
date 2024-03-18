// Next steps:
//   1. Add ros
//      1.1 Add status publisher

// Network socket stuff
#include <net/if.h>
#include <fcntl.h>

// ROS
#include "rclcpp/rclcpp.hpp"

// RoverCanLib
#include "rover_can_lib/config.hpp"
#include "rover_can_lib/can_device.hpp"
#include "rover_can_lib/msgs/heartbeat.hpp"
#include "rover_can_lib/msgs/propulsion_motor.hpp"

#define LOGGER_NAME "CanMasterNode"

int createSocket(const char *canNetworkName_);
RoverCanLib::Constant::eInternalErrorCode readMsgFromCanSocket(int canSocket_, Chrono<uint64_t, millis> *chonoCanWatchdog_);
void CB_Can_PropulsionMotor(uint16_t id_, const can_frame *frameMsg);

volatile sig_atomic_t shutdownFlag = 0;
void signal_handler(int signo);

// Global msgs
RoverCanLib::Msgs::PropulsionMotor msg_0x101;

// Global hash map of msgs
std::unordered_map<int, RoverCanLib::Msgs::Msg *> msgsMap{
    {(size_t)RoverCanLib::Constant::eDeviceId::FRONTLEFT_MOTOR, &msg_0x101}};

// Global hash map of devices
std::unordered_map<int, CanDevice> deviceMap{
    {(size_t)RoverCanLib::Constant::eDeviceId::FRONTLEFT_MOTOR, CanDevice(0x101u, CB_Can_PropulsionMotor)}};

int main(int argc, char **argv)
{
    for (EVER)
    {
        int canSocket = createSocket("canRovus");
        assert(canSocket > 0);

        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Canbus ready, starting loop");

        RoverCanLib::Msgs::Heartbeat msgHeartbeat;
        Timer<uint64_t, millis> timerHeartbeat((uint64_t)(1000.0f / RoverCanLib::Constant::HEARTBEAT_FREQ));
        TimerFixedLoop<std::chrono::microseconds> loopTimer(std::chrono::microseconds(10));
        Chrono<uint64_t, millis> chonoCanWatchdog;
        for (EVER)
        {
            // Send heartbeat at X Hz
            if (timerHeartbeat.isDone())
            {
                can_frame canFrame;
                canFrame.can_id = (canid_t)RoverCanLib::Constant::eDeviceId::MASTER_COMPUTER_UNIT;
                msgHeartbeat.getMsg((uint8_t)RoverCanLib::Msgs::Heartbeat::eMsgID::DONT_USE, &canFrame, rclcpp::get_logger(LOGGER_NAME));
#warning TODO Add check on write
                write(canSocket, &canFrame, sizeof(canFrame));
            }

            // Checking watchdog
            if (chonoCanWatchdog.getTime() > RoverCanLib::Constant::WATCHDOG_TIMEOUT_MS)
            {
                RCLCPP_FATAL(rclcpp::get_logger(LOGGER_NAME), "Watchdog triggered, trying to reconnect to the can network...");
                break;
            }

            // Empty rx queue
            if (readMsgFromCanSocket(canSocket, &chonoCanWatchdog) != RoverCanLib::Constant::eInternalErrorCode::OK)
            {
                RCLCPP_FATAL(rclcpp::get_logger(LOGGER_NAME), "Error detected on CAN socket, trying to reconnect to the network...");
                break;
            }

            loopTimer.sleepUntilReady();
        }

        close(canSocket);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    return 0;
}

// Returns a socket on success or -1 on error
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

// Read all msg in the rx queue and parse them accordingly. Also resets watchdog if msg is received
RoverCanLib::Constant::eInternalErrorCode readMsgFromCanSocket(int canSocket_, Chrono<uint64_t, millis> *chonoCanWatchdog_)
{
    for (EVER)
    {
        struct can_frame frame;
        ssize_t bytes_read;
        bytes_read = read(canSocket_, &frame, sizeof(struct can_frame));

        if (bytes_read == sizeof(can_frame))
        {
            chonoCanWatchdog_->restart();
            // Check if the device exist in the hash map and parse can msg
            if (deviceMap.find((size_t)frame.can_id) != deviceMap.end())
            {
                deviceMap.find((size_t)frame.can_id)->second.parseMsg(&frame);
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
        msgsMap.find(id_)->second->parseMsg(frameMsg, rclcpp::get_logger(LOGGER_NAME));
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                     "Received unexpected msg id: 0x%.2x Possible mismatch in library version between nodes",
                     frameMsg->data[(size_t)RoverCanLib::Constant::eDataIndex::MSG_ID]);
    }
}
