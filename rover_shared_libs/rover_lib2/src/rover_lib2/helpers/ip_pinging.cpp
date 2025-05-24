#include "ip_pinging.hpp"

#if defined(__linux__)
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <regex>
#endif  // defined(__linux__)

namespace RoverLib2
{
#if defined(__linux__)

    RtspUrlInfo parseRtspUrl(const std::string& url)
    {
        RtspUrlInfo info = {"", 554, "", false};

        std::regex rtsp_regex(R"(^rtsp://(?:[^@/]+@)?([^:/]+)(?::(\d+))?(/.*)?$)");
        std::smatch matches;

        if (std::regex_match(url, matches, rtsp_regex))
        {
            info.host = matches[1].str();

            if (matches[2].matched)
            {
                info.port = std::stoi(matches[2].str());
            }

            info.path = matches[3].matched ? matches[3].str() : "/";
            info.valid = true;
        }

        return info;
    }

    bool isIPReachable(const std::string& rtspUrl, size_t timeoutMs)
    {
        RtspUrlInfo urlInfo = parseRtspUrl(rtspUrl);
        if (!urlInfo.valid)
        {
            return false;
        }

        std::string ip = urlInfo.host;
        size_t port = urlInfo.port;

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

        if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0)
        {
            close(sock);
            return false;
        }

        sockaddr_in addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);

        if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1)
        {
            struct addrinfo hints, *res;
            std::memset(&hints, 0, sizeof(hints));
            hints.ai_family = AF_INET;
            hints.ai_socktype = SOCK_STREAM;

            int err = getaddrinfo(ip.c_str(), nullptr, &hints, &res);
            if (err != 0)
            {
                close(sock);
                return false;
            }

            memcpy(&addr.sin_addr, &((struct sockaddr_in*)res->ai_addr)->sin_addr, sizeof(struct in_addr));
            freeaddrinfo(res);
        }

        int result = connect(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
        if (result < 0 && errno != EINPROGRESS)
        {
            close(sock);
            return false;
        }

        fd_set writefds, errorfds;
        FD_ZERO(&writefds);
        FD_ZERO(&errorfds);
        FD_SET(sock, &writefds);
        FD_SET(sock, &errorfds);

        struct timeval tv;
        tv.tv_sec = timeoutMs / 1000;
        tv.tv_usec = (timeoutMs % 1000) * 1000;

        result = select(sock + 1, nullptr, &writefds, &errorfds, &tv);

        if (result <= 0)
        {
            close(sock);
            return false;
        }

        if (FD_ISSET(sock, &errorfds))
        {
            close(sock);
            return false;
        }

        if (FD_ISSET(sock, &writefds))
        {
            int so_error = 0;
            socklen_t len = sizeof(so_error);
            if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &len) < 0)
            {
                close(sock);
                return false;
            }

            close(sock);
            return (so_error == 0);
        }

        close(sock);
        return false;
    }

#endif  // defined(__linux__)
}  // namespace RoverLib2