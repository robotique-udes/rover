// Flow
//  Empty rx queue and call each callback accordingly (one callback byID) --> They should only modify values inside ros defined messages
//  if device doesn't exist already, add it to the device array
//      If watchdog, clear its watchdog --> Helper
//      If error set error state --> Helper
//      If other do specific actions --> Custom func
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <cstdint>
#include <chrono>
#include <thread>

#include <signal.h>

#include "rover_can_lib/union_type_definition.hpp"
#include "rover_can_lib/constant.hpp"
#include "rovus_lib/macros.h"
#include "rovus_lib/timer.hpp"

#include <iostream>
#include <bitset>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/can_device_status.hpp"

#define TIME_WATCHDOG 500

volatile sig_atomic_t shutdownFlag = 0;
void signal_handler(int signo);

class CanDevice
{
public:
    CanDevice(uint16_t id_ /*, rclcpp::Publisher<rover_msgs::msg::CanDeviceStatus>::SharedPtr pub_CanBusState_*/)
    {
        _id = id_;
        /*_pub_CanBusState = pub_CanBusState_;*/
    }
    ~CanDevice() {}

    void parseMsg(uint8_t *data_, uint8_t dataLen_)
    {
        switch (data_[(uint8_t)RoverCanLib::Constant::eDataIndex::MSG_ID])
        {
        case (uint8_t)RoverCanLib::Constant::eMsgId::HEARTBEAT:
            RCLCPP_INFO(this->_getLogger(), "Received heartbeat!");
            if (_timerWatchdog.getTime() > TIME_WATCHDOG)
            {
                _msg_canStatus.watchdog_ok = false;
            }
            else
            {
                _msg_canStatus.watchdog_ok = true;
            }
            _timerWatchdog.restart();
            break;

        case (uint8_t)RoverCanLib::Constant::eMsgId::ERROR_STATE:
#warning TODO
            

            RCLCPP_WARN(this->_getLogger(),
                        "ERROR_STATE isn't impleted yet, but should be!!!",
                        (uint8_t)RoverCanLib::Constant::eDataIndex::MSG_ID);

        default:
            RCLCPP_ERROR(this->_getLogger(),
                         "Received unexpected msg id: 0x%.3x Possible mismatch in library version between nodes",
                         data_[(uint8_t)RoverCanLib::Constant::eDataIndex::MSG_ID]);
        }
    }

private:
    uint16_t _id;
    // rclcpp::Publisher<rover_msgs::msg::CanDeviceStatus>::SharedPtr _pub_CanBusState;
    rover_msgs::msg::CanDeviceStatus _msg_canStatus;
    Chrono<uint64_t, millis> _timerWatchdog;

    rclcpp::Logger _getLogger()
    {
        std::stringstream ss;
        ss << std::hex << _id; // Set the stream to output in hexadecimal
        return rclcpp::get_logger("0x" + ss.str());
    }
};

int main(int argc, char **argv)
{
    int s;
    struct sockaddr_can addr;
    struct can_frame frame;

    // Create socket
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("socket");
        return 1;
    }

    // Bind socket to the CAN interface
    addr.can_family = AF_CAN;
    addr.can_ifindex = if_nametoindex("canRovus"); // Replace "can0" with your CAN interface
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        close(s);
        return 1;
    }

    // Set socket to non-blocking mode
    if (fcntl(s, F_SETFL, O_NONBLOCK) < 0)
    {
        perror("fcntl");
        close(s);
        return 1;
    }

    CanDevice d_0x101(0x101);

    // Perform non-blocking read
    ssize_t bytes_read;
    while (1)
    {
        bytes_read = read(s, &frame, sizeof(struct can_frame));
        if (bytes_read == sizeof(struct can_frame))
        {
            printf("\nReceived data, id: %x\n", frame.can_id);
            if (frame.can_id == (uint16_t)RoverCanLib::Constant::eDeviceId::FRONTLEFT_MOTOR)
            {
                d_0x101.parseMsg(frame.data, frame.can_dlc);
            }
            // Data received successfully, process it
        }
        else if (bytes_read == -1)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // printf("-");
                // No data available, continue or do other work
                // usleep(10000); // Sleep briefly before trying again
                continue;
            }
            else
            {
                perror("read");
                break;
            }
        }
    }

    close(s);
    return 0;
}

// int main(void)
// {
//     int s;
//     sockaddr_can addr;
//     ifreq ifr;
//     can_frame frame;

//     printf("CAN Sockets Demo\r\n");

//     if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
//     {
//         perror("Socket");
//         return 1;
//     }

//     strcpy(ifr.ifr_name, "canRovus");
//     ioctl(s, SIOCGIFINDEX, &ifr);

//     memset(&addr, 0, sizeof(addr));
//     addr.can_family = AF_CAN;
//     addr.can_ifindex = ifr.ifr_ifindex;

//     if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
//     {
//         perror("Bind");
//         return 1;
//     }

//     frame.can_id = 0x101;
//     frame.can_dlc = 5;
//     frame.data[0] = (uint8_t)RoverCanLib::Constant::eMsgId::PROPULSION_MOTOR;

//     RoverCanLib::UnionDefinition::FloatUnion test;
//     for (float value = -100.0f; value <= 100.0f; value += 0.1f)
//     {
//         test.data = value;

//         for (uint8_t i = 0; i < sizeof(test); i++)
//         {
//             frame.data[i + 1] = test.dataBytes[i];
//         }

//         write(s, &frame, sizeof(frame)) != sizeof(frame);
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));

//     }
//     if (close(s) < 0)
//     {
//         perror("Close");
//         return 1;
//     }

//     return 0;
// }
