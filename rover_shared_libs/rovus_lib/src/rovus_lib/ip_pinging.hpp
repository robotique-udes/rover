#ifndef IP_PINGING
#define IP_PINGING

#include <string>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <unistd.h>

namespace RoverLib
{
    bool isIPReachable(const std::string& ip_, size_t port_, size_t timeoutMs_ = 500U);
}

#endif  // IP_PINGING