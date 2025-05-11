#include "ip_pinging.hpp"

#if defined(__linux__)
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
    bool isIPReachable(const std::string& ip_, size_t port_, size_t timeoutMs_)
    {
        size_t start = ip_.find("rtsp://");
        if (start == std::string::npos)
        {
            return false;
        }
        start += 7;

        if (start >= ip_.length())
        {
            return false;
        }

        std::string ip;
        size_t atSymbol = ip_.find('@', start);
        if (atSymbol != std::string::npos)
        {
            size_t ipStart = atSymbol + 1;
            if (ipStart >= ip_.length())
            {
                return false;
            }
            size_t ipEnd = ip_.find_first_of(":/", ipStart);
            if (ipEnd == std::string::npos)
            {
                ip = ip_.substr(ipStart);
            }
            else
            {
                ip = ip_.substr(ipStart, ipEnd - ipStart);
            }
        }
        else
        {
            size_t end = ip_.find_first_of(":/", start);
            if (end == std::string::npos)
            {
                ip = ip_.substr(start);
            }
            else
            {
                ip = ip_.substr(start, end - start);
            }
        }

        if (ip.empty())
        {
            return false;
        }

        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0)
        {
            return false;
        }

        int flags = fcntl(sock, F_GETFL, 0);
        if (flags < 0)
        {
            close(sock);
            return false;
        }

        fcntl(sock, F_SETFL, flags | O_NONBLOCK);

        sockaddr_in addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port_);

        if (inet_addr(ip.c_str()) == INADDR_NONE)
        {
            struct addrinfo hints, *res;
            std::memset(&hints, 0, sizeof(hints));
            hints.ai_family = AF_INET;
            int err = getaddrinfo(ip.c_str(), nullptr, &hints, &res);
            if (err != 0)
            {
                close(sock);
                return false;
            }
            memcpy(&addr.sin_addr, &((struct sockaddr_in*)res->ai_addr)->sin_addr, sizeof(struct in_addr));
            freeaddrinfo(res);
        }
        else
        {
            addr.sin_addr.s_addr = inet_addr(ip.c_str());
        }

        int result = connect(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
        if (result < 0 && errno != EINPROGRESS)
        {
            close(sock);
            return false;
        }

        fd_set writefds;
        FD_ZERO(&writefds);
        FD_SET(sock, &writefds);

        struct timeval tv;
        tv.tv_sec = timeoutMs_ / 1000;
        tv.tv_usec = (timeoutMs_ % 1000) * 1000;

        result = select(sock + 1, nullptr, &writefds, nullptr, &tv);
        if (result <= 0)
        {
            close(sock);
            return false;
        }

        int so_error = 0;
        socklen_t len = sizeof(so_error);
        getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &len);
        close(sock);

        return so_error == 0;
    }
#endif  // defined(__linux__)
}  // namespace RoverLib2