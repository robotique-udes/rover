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

#define PORT 1234
#define MAXLINE 1024

class ClientUDPAntenna : public rclcpp::Node
{
public:
    ClientUDPAntenna();
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

    sockUDP createSocket();

    rover_msgs::msg::AntennaCmd _msgCbAbtr;
    char _bufferSend[MAXLINE] = {'\0'};
    char _bufferRecv[MAXLINE] = {'\0'};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClientUDPAntenna>());
    rclcpp::shutdown();
    return 0;
}

ClientUDPAntenna::ClientUDPAntenna() : Node("clientUDPAntenna")
{
    _pub_gps_antenna = this->create_publisher<rover_msgs::msg::AntennaCmd>("/base/antenna/cmd/in/auto", 1);
    _sub_abtr = this->create_subscription<rover_msgs::msg::AntennaCmd>("/base/antenna/cmd/out/goal",
                                                                     1,
                                                                     [this](const rover_msgs::msg::AntennaCmd msg)
                                                                     { callbackAbtr(msg); });
    _timer_recv = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ClientUDPAntenna::cbTimerRecv, this));
    _timer_send = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ClientUDPAntenna::cbTimerSend, this));

    // int sockfd;
    // char bufferSend[MAXLINE] = "ping";
    // char bufferRecv[MAXLINE] = {'\0'};

    // // Creating socket file descriptor
    // if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    //     perror("socket creation failed");
    //     exit(EXIT_FAILURE);
    // }

    // struct sockaddr_in servAddr;
    // socklen_t sLen = sizeof(servAddr);

    // servAddr.sin_family = AF_INET;
    // servAddr.sin_port = htons(PORT);
    // servAddr.sin_addr.s_addr = inet_addr("192.168.144.100");

    // sockUDP sockCreated = createSocket();
    // int sockfd = sockCreated.socketUDP;
    // struct sockaddr_in servAddr = sockCreated.servAddr;
    // socklen_t sLen = sockCreated.sLen;


    // // Sending a message to the server
    // ssize_t sByte = sendto(sockfd, bufferSend, sizeof(bufferSend), 0, (struct sockaddr *)&servAddr, sLen);
    // if (sByte < 0) {
    //     printf("sendto failed\n");
    //     close(sockfd);
    //     exit(EXIT_FAILURE);
    // }
    // std::cout << "[" << sByte << "] Bytes Sent: " << bufferSend << std::endl;

    // // Receiving a response from the server
    // ssize_t rByte = recvfrom(sockfd, bufferRecv, sizeof(bufferRecv), 0, (struct sockaddr *)&servAddr, &sLen);
    // if (rByte < 0) {;
    //     printf("recvfrom failed\n");
    //     close(sockfd);
    //     exit(EXIT_FAILURE);
    // }
    // std::cout << "[" << rByte << "] Bytes Received: " << bufferRecv << std::endl;

    // close(sockfd);
}

ClientUDPAntenna::sockUDP ClientUDPAntenna::createSocket()
{
    sockUDP sock;

    // Creating socket file descriptor
    if ((sock.socketUDP = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    sock.servAddr.sin_family = AF_INET;
    sock.servAddr.sin_port = htons(PORT);
    sock.servAddr.sin_addr.s_addr = inet_addr("192.168.144.100");

    return sock;

}

void ClientUDPAntenna::callbackAbtr(const rover_msgs::msg::AntennaCmd msg_)
{
    _msgCbAbtr = msg_;
    // _bufferSend = std::to_chars(msg_.speed);
    std::sprintf(_bufferSend, "%.2f", _msgCbAbtr.speed); 
}

void ClientUDPAntenna::cbTimerRecv()
{
    
}

void ClientUDPAntenna::cbTimerSend()
{
     sockUDP sockCreated = createSocket();
    int sockfd = sockCreated.socketUDP;
    struct sockaddr_in servAddr = sockCreated.servAddr;
    socklen_t sLen = sockCreated.sLen;

    // Sending a message to the server
    ssize_t sByte = sendto(sockfd, _bufferSend, sizeof(_bufferSend), 0, (struct sockaddr *)&servAddr, sLen);
    if (sByte < 0) {
        printf("sendto failed\n");
        close(sockfd);
        exit(EXIT_FAILURE);
    }
    std::cout << "[" << sByte << "] Bytes Sent: " << _bufferSend << std::endl;

    close(sockfd);
}
