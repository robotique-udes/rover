#ifndef IP_PINGING
#define IP_PINGING

#if defined(__linux__)
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <unistd.h>
#endif  // defined(__linux__)

namespace RoverLib2
{
#if defined(__linux__)
    bool isIPReachable(const std::string& ip_, size_t port_, size_t timeoutMs_ = 500U);
#endif  // defined(__linux__)
}  // namespace RoverLib2

#endif  // IP_PINGING