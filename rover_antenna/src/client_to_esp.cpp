#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define PORT 1234
#define MAXLINE 1024

int main() {
    int sockfd;
    char bufferSend[MAXLINE] = "ping";  // Assuming the client sends "ping"
    char bufferRecv[MAXLINE] = {'\0'};

    // Creating socket file descriptor
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    struct sockaddr_in servAddr;
    socklen_t sLen = sizeof(servAddr);

    servAddr.sin_family = AF_INET;
    servAddr.sin_port = htons(PORT);
    servAddr.sin_addr.s_addr = inet_addr("192.168.144.100");

    // Sending a message to the server
    ssize_t sByte = sendto(sockfd, bufferSend, sizeof(bufferSend), 0, (struct sockaddr *)&servAddr, sLen);
    if (sByte < 0) {
        printf("sendto failed\n");
        close(sockfd);
        exit(EXIT_FAILURE);
    }
    std::cout << "[" << sByte << "] Bytes Sent: " << bufferSend << std::endl;

    // Receiving a response from the server
    ssize_t rByte = recvfrom(sockfd, bufferRecv, sizeof(bufferRecv), 0, (struct sockaddr *)&servAddr, &sLen);
    if (rByte < 0) {
        printf("recvfrom failed\n");
        close(sockfd);
        exit(EXIT_FAILURE);
    }
    std::cout << "[" << rByte << "] Bytes Received: " << bufferRecv << std::endl;

    close(sockfd);
    return 0;
}