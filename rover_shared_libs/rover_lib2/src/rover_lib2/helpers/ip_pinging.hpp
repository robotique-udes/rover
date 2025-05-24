#ifndef IP_PINGING
#define IP_PINGING

#if defined(__linux__)
#include <string>
#include <cstddef>
#endif  // defined(__linux__)

namespace RoverLib2
{
#if defined(__linux__)

    struct RtspUrlInfo
    {
        std::string host;
        int port;
        std::string path;
        bool valid;
    };

    RtspUrlInfo parseRtspUrl(const std::string& url);

    bool isIPReachable(const std::string& rtspUrl, size_t timeoutMs = 5000);

#endif  // defined(__linux__)
}  // namespace RoverLib2

#endif  // IP_PINGING