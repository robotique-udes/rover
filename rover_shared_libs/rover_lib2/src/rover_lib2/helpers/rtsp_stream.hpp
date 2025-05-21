#ifndef RTSP_STREAM
#define RTSP_STREAM

#if defined(__linux__)
#include <string>
#endif  // defined(__linux__)

namespace RoverLib2
{
#if defined(__linux__)
    bool hasRTSPStream(const std::string& ip_, size_t timeoutMs_ = 500U);
#endif  // defined(__linux__)
}  // namespace RoverLib2

#endif  // IP_PINGING