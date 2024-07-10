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
#define MAXLINE 255

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

    struct sockUDP
    {
        int socketUDP;
        struct sockaddr_in servAddr;
        socklen_t sLen = sizeof(servAddr);
    };

    // sockUDP sock;

    int _socket;
    struct sockaddr_in _servAddr;
    socklen_t _sLen = sizeof(_servAddr);

    Timer<uint64_t, millis> timerSend = Timer<uint64_t, millis>(500u);

    rover_msgs::msg::AntennaCmd _msgCbAbtr;
    char _bufferSend[MAXLINE] = {'\0'};
    char _bufferRecv[MAXLINE] = {'\0'};
};

int main(int argc, char *argv[])
{
    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<ClientUDPAntenna>());
    // rclcpp::shutdown();
    // return 0;
    // int sockfd;
    // char bufferSend[] = "ping";
    // char bufferRecv[MAXLINE] = {'\0'};

    // // Creating socket file descriptor
    // if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    // {
    //     perror("socket creation failed");
    //     exit(EXIT_FAILURE);
    // }
    // struct sockaddr_in servAddr;
    // socklen_t sLen = sizeof(servAddr);

    // servAddr.sin_family = AF_INET;
    // servAddr.sin_port = htons(PORT);
    // servAddr.sin_addr.s_addr = inet_addr("192.168.144.50");

    // if (bind(sockfd, (struct sockaddr *)&servAddr, sizeof(servAddr)) < 0)
    // {
    //     perror("Connection error");
    // }

    // Timer<uint64_t, millis> timerSend(500u);

    // for (EVER)
    // {
    //     if (timerSend.isDone())
    //     {
    //         // Sending a message to the server
    //         ssize_t sByte = sendto(sockfd, bufferSend, sizeof(bufferSend), 0, (sockaddr *)&servAddr, sLen);
    //         if (sByte < 0)
    //         {
    //             printf("sendto failed\n");
    //             close(sockfd);
    //             exit(EXIT_FAILURE);
    //         }
    //         std::cout << "[" << sByte << "] Bytes Sent: " << bufferSend << std::endl;
    //     }

    //     RCLCPP_INFO(rclcpp::get_logger(""), "Before receive");
    //     int64_t byteRecv = (int64_t)recv(sockfd, bufferRecv, sizeof(bufferRecv), 0);
    //     RCLCPP_INFO(rclcpp::get_logger(""), "Bytes : %s", bufferRecv);
    // }
    signal(SIGINT, signal_handler); // Makes CTRL+C work

    rclcpp::init(argc, argv);
    while (!g_shutdownFlag)
    {
        int sock;
        char bufferSend[] = "ping";
        char bufferRecv[MAXLINE] = {'\0'};

        // Creating socket file descriptor
        if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        {
            perror("socket creation failed");
        }

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        RCLCPP_INFO(rclcpp::get_logger(""), "%i", setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv));

        struct sockaddr_in servAddr;
        socklen_t sLen = sizeof(servAddr);

        servAddr.sin_family = AF_INET;
        servAddr.sin_port = htons(PORT);
        servAddr.sin_addr.s_addr = inet_addr("192.168.144.50");

        if (bind(sock, (struct sockaddr *)&servAddr, sizeof(servAddr)) < 0)
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
    // _timer_send = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ClientUDPAntenna::cbTimerSend, this));
    _timer_send = this->create_wall_timer(std::chrono::microseconds(1), std::bind(&ClientUDPAntenna::cbTimerSend, this));

    _socket = sock;
    _servAddr = servAddr;
    _sLen = sLen;
}

void ClientUDPAntenna::callbackAbtr(const rover_msgs::msg::AntennaCmd msg_)
{
    _msgCbAbtr = msg_;
    // _bufferSend = std::to_chars(msg_.speed);
    std::sprintf(_bufferSend, "%.2f", _msgCbAbtr.speed);
}

void ClientUDPAntenna::cbTimerSend()
{
    // Sending a message to the server
    if (timerSend.isDone())
    {
        ssize_t sByte = sendto(_socket, _bufferSend, sizeof(_bufferSend), 0, (struct sockaddr *)&_servAddr, _sLen);
        if (sByte < 0)
        {
            printf("sendto failed\n");
        }
        std::cout << "[" << sByte << "] Bytes Sent: " << _bufferSend << std::endl;
    }

    // while (true)
    // {
        int64_t rByte = (int64_t)recv(_socket, _bufferRecv, sizeof(_bufferRecv), 0);
        if (rByte == 0)
        {
            // break;
        }
        else if (rByte < 0)
        {
            printf("sendto failed\n");
        }
        RCLCPP_INFO(rclcpp::get_logger(""), "Bytes : %s", _bufferRecv);
    // }
}
