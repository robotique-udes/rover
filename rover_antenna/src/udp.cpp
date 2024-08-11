#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/antenna_cmd.hpp"
#include "rover_msgs/msg/gps.hpp"
#include "rovus_lib/macros.h"
#include "rovus_lib/timer.hpp"

#define PORT 1234
#define IP_LOCAL "192.168.140.100"
#define IP_SERVER "192.168.140.60"

void signalHandler(int signo_);
volatile sig_atomic_t g_shutdownFlag = 0;

class ClientUDPAntenna : public rclcpp::Node
{
    struct sMsgAbtrCmd
    {
        float speed = 0.0f;
        bool enable = 0.0f;
    };

    struct sMsgGPS
    {
        float latitude = 0.0f;
        float longitude = 0.0f;
    };

public:
    ClientUDPAntenna(int sock_, struct sockaddr_in servAddr_, socklen_t sLen_);
    ~ClientUDPAntenna() {}

private:
    void callbackAbtr(const rover_msgs::msg::AntennaCmd msg_);
    void cbTimerMsg();

    rclcpp::Publisher<rover_msgs::msg::Gps>::SharedPtr _pub_gpsAntenna;
    rclcpp::Subscription<rover_msgs::msg::AntennaCmd>::SharedPtr _sub_abtr;

    rclcpp::TimerBase::SharedPtr _timerMsg;

    int _socket;
    struct sockaddr_in _servAddr;
    socklen_t _sLen = sizeof(_servAddr);

    Timer<uint64_t, millis> _timerSend = Timer<uint64_t, millis>(200u);

    sMsgAbtrCmd _msgCbAbtr;
    sMsgAbtrCmd _bufferSend;
    uint8_t _bufferRecv[sizeof(sMsgAbtrCmd)] = {0};
    rover_msgs::msg::Gps _gpsAntenna;
};

int main(int argc, char *argv[])
{
    signal(SIGINT, signalHandler); // Makes CTRL+C work

    rclcpp::init(argc, argv);
    while (!g_shutdownFlag)
    {
        int sock;

        // Creating socket file descriptors
        if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("udp_client(not init)"), "Sockect creation failed, retrying to connect");
            close(sock);
            continue;
        }

        // sets timeout of receive to 0 so it's not blocking
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv) < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("udp_client(not init)"), "Socket option failed, retying to connect");
            close(sock);
            continue;
        }

        struct sockaddr_in servAddr;
        socklen_t sLen = sizeof(servAddr);

        servAddr.sin_family = AF_INET;
        servAddr.sin_port = htons(PORT);
        servAddr.sin_addr.s_addr = inet_addr(IP_LOCAL);

        if (bind(sock, (struct sockaddr *)&servAddr, sizeof(servAddr)) < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("udp_client(not init)"), "Connection to server failed, try connecting to roverAntenna network");
            close(sock);
            continue;
        }

        servAddr.sin_addr.s_addr = inet_addr(IP_SERVER);

        rclcpp::spin(std::make_shared<ClientUDPAntenna>(sock, servAddr, sLen));

        close(sock);
        sleep(1);
    }
}

void signalHandler(int signo_)
{
    RCLCPP_INFO(rclcpp::get_logger("[SHUTDOWN]"), "Shutdown asked by user, exiting...");
    (void)signo_;
    g_shutdownFlag = 1;
}

ClientUDPAntenna::ClientUDPAntenna(int sock_, struct sockaddr_in servAddr_, socklen_t sLen_) : Node("clientUDPAntenna")
{
    _socket = sock_;
    _servAddr = servAddr_;
    _sLen = sLen_;

    _pub_gpsAntenna = this->create_publisher<rover_msgs::msg::Gps>("/base/antenna/gps/position", 1);
    _sub_abtr = this->create_subscription<rover_msgs::msg::AntennaCmd>("/base/antenna/cmd/out/goal",
                                                                       1,
                                                                       [this](const rover_msgs::msg::AntennaCmd msg)
                                                                       { callbackAbtr(msg); });
    _timerMsg = this->create_wall_timer(std::chrono::microseconds(10), std::bind(&ClientUDPAntenna::cbTimerMsg, this));
}

void ClientUDPAntenna::callbackAbtr(const rover_msgs::msg::AntennaCmd msg_)
{
    _msgCbAbtr.enable = msg_.enable;
    _msgCbAbtr.speed = msg_.speed;
}

void ClientUDPAntenna::cbTimerMsg()
{
    // Sending a message to the server
    if (_timerSend.isDone())
    {
        ssize_t sByte = sendto(_socket, &_msgCbAbtr, sizeof(sMsgAbtrCmd), 0, (struct sockaddr *)&_servAddr, _sLen);
        if (sByte < 0)
        {
            RCLCPP_WARN(rclcpp::get_logger(""), "sendto failed");
        }
        RCLCPP_INFO(rclcpp::get_logger(""), "Message sent : %f", _msgCbAbtr.speed);
    }

    // int64_t recvByte = (int64_t)recv(_socket, _bufferRecv, sizeof(_bufferRecv), 0);
    // if (recvByte < 0)
    // {
    //     RCLCPP_WARN(rclcpp::get_logger(""), "received failed");
    // }
    // else
    // {
    //     _gpsAntenna.latitude = ((sMsgGPS *)_bufferRecv)->latitude;
    //     _gpsAntenna.longitude = ((sMsgGPS *)_bufferRecv)->longitude;
    //     _pub_gpsAntenna->publish(_gpsAntenna);

    //     // RCLCPP_INFO(rclcpp::get_logger(""), "Message received : %f", _gpsAntenna.latitude);
    // }
}
