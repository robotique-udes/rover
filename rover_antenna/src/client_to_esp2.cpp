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
    char bufferRecv[MAXLINE];
    char bufferSend[MAXLINE] = "pong";

    // Creating socket file descriptor
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    struct sockaddr_in servAddr, cliAddr;
    socklen_t cliLen = sizeof(cliAddr);

    // Filling server information
    servAddr.sin_family = AF_INET; // IPv4
    servAddr.sin_addr.s_addr = INADDR_ANY; // Any IP address
    servAddr.sin_port = htons(PORT); // Server port

    // Bind the socket with the server address
    if (bind(sockfd, (const struct sockaddr *)&servAddr, sizeof(servAddr)) < 0) {
        perror("bind failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    printf("Server is ready to receive on port %d\n", PORT);

    // Receive message from client
    ssize_t rByte = recvfrom(sockfd, (char *)bufferRecv, MAXLINE, 0, (struct sockaddr *)&cliAddr, &cliLen);
    if (rByte < 0) {
        printf("recvfrom failed\n");
        close(sockfd);
        exit(EXIT_FAILURE);
    }
    bufferRecv[rByte] = '\0'; // Null-terminate the received string
    std::cout << "[" << rByte << "] Bytes Received: " << bufferRecv << std::endl;

    // Send response to client
    ssize_t sByte = sendto(sockfd, bufferSend, strlen(bufferSend), 0, (struct sockaddr *)&cliAddr, cliLen);
    if (sByte < 0) {
        printf("sendto failed\n");
        close(sockfd);
        exit(EXIT_FAILURE);
    }
    std::cout << "[" << sByte << "] Bytes Sent: " << bufferSend << std::endl;

    close(sockfd);
    return 0;
}