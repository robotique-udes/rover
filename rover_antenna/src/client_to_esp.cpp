#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/antenna_cmd.hpp"
#include "rovus_lib/macros.h"
#include "rovus_lib/timer.hpp"

#define PORT 1234
#define MAXLEN 255

void signal_handler(int signo_);
volatile sig_atomic_t g_shutdownFlag = 0;

class ClientUDPAntenna : public rclcpp::Node
{
public:
    ClientUDPAntenna(int sock, struct sockaddr_in servAddr, socklen_t sLen);
    ~ClientUDPAntenna() {}

private:
    void callbackAbtr(const rover_msgs::msg::AntennaCmd msg_);
    void sendGpsAntenna();
    void cbTimerRecv();
    void cbTimerSend();

    rclcpp::Publisher<rover_msgs::msg::AntennaCmd>::SharedPtr _pub_gps_antenna;
    rclcpp::Subscription<rover_msgs::msg::AntennaCmd>::SharedPtr _sub_abtr;

    rclcpp::TimerBase::SharedPtr _timer_recv;
    rclcpp::TimerBase::SharedPtr _timer_send;

    struct MsgAbtrCmd
    {
        float speed;
        bool enable;
    };

    struct MsgGPS
    {
        float lattitude;
        float longitude;
    };

    struct sockUDP
    {
        int socketUDP;
        struct sockaddr_in servAddr;
        socklen_t sLen = sizeof(servAddr);
    };

    int _socket;
    struct sockaddr_in _servAddr;
    socklen_t _sLen = sizeof(_servAddr);

    Timer<uint64_t, millis> timerSend = Timer<uint64_t, millis>(200u);

    MsgAbtrCmd _msgCbAbtr;
    MsgAbtrCmd _bufferSend;
    char _bufferRecv[MAXLEN] = {'\0'};
};

int main(int argc, char *argv[])
{
    signal(SIGINT, signal_handler); // Makes CTRL+C work

    rclcpp::init(argc, argv);
    while (!g_shutdownFlag)
    {
        int sock;

        // Creating socket file descriptor
        if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        {
            perror("socket creation failed");
        }

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        RCLCPP_INFO(rclcpp::get_logger(""), "socketopt : %i", setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv));

        struct sockaddr_in servAddr;
        socklen_t sLen = sizeof(servAddr);

        servAddr.sin_family = AF_INET;
        servAddr.sin_port = htons(PORT);
        servAddr.sin_addr.s_addr = inet_addr("192.168.144.50");

        while (bind(sock, (struct sockaddr *)&servAddr, sizeof(servAddr)) < 0)
        {
            perror("Connection error");
        }

        servAddr.sin_addr.s_addr = inet_addr("192.168.144.100");

        rclcpp::spin(std::make_shared<ClientUDPAntenna>(sock, servAddr, sLen));

        close(sock);
        sleep(1);
    }
}

void signal_handler(int signo_)
{
    RCLCPP_INFO(rclcpp::get_logger("[SHUTDOWN]"), "Shutdown asked by user, exiting...");
    (void)signo_;
    g_shutdownFlag = 1;
}

ClientUDPAntenna::ClientUDPAntenna(int sock, struct sockaddr_in servAddr, socklen_t sLen) : Node("clientUDPAntenna")
{
    _pub_gps_antenna = this->create_publisher<rover_msgs::msg::AntennaCmd>("/base/antenna/cmd/in/auto", 1);
    _sub_abtr = this->create_subscription<rover_msgs::msg::AntennaCmd>("/base/antenna/cmd/out/goal",
                                                                       1,
                                                                       [this](const rover_msgs::msg::AntennaCmd msg)
                                                                       { callbackAbtr(msg); });
    _timer_send = this->create_wall_timer(std::chrono::microseconds(10), std::bind(&ClientUDPAntenna::cbTimerSend, this));

    _socket = sock;
    _servAddr = servAddr;
    _sLen = sLen;
}

void ClientUDPAntenna::callbackAbtr(const rover_msgs::msg::AntennaCmd msg_)
{
    _msgCbAbtr.enable = msg_.enable;
    _msgCbAbtr.speed = msg_.speed;
    RCLCPP_INFO(rclcpp::get_logger(""), "Callback Abtr : %f", _msgCbAbtr.speed);
}

void ClientUDPAntenna::cbTimerSend()
{

    // Sending a message to the server
    if (timerSend.isDone())
    {
        ssize_t sByte = sendto(_socket, &_msgCbAbtr, sizeof(MsgAbtrCmd), 0, (struct sockaddr *)&_servAddr, _sLen);
        if (sByte < 0)
        {
            printf("sendto failed\n");
        }
        RCLCPP_INFO(rclcpp::get_logger(""), "Bytes send : %f", _msgCbAbtr.speed);
    }

    // RCLCPP_INFO(rclcpp::get_logger(""), "Before receiving");
    int64_t recvByte = (int64_t)recv(_socket, _bufferRecv, sizeof(_bufferRecv), 0);
    if (recvByte == 0)
    {
        // break;
    }
    else if (recvByte < 0)
    {
        printf("sendto failed\n");
    }
    MsgGPS *receivedData = (MsgGPS *)_bufferRecv;
    // RCLCPP_INFO(rclcpp::get_logger(""), "Bytes : %f", receivedData->lattitude);
}
