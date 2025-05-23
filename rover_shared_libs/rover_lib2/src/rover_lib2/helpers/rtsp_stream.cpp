// rtsp_stream.cpp
#include "rtsp_stream.hpp"

#if defined(__linux__)
  #include <string>
  #include <cstring>
  #include <cerrno>
  #include <sys/types.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <netdb.h>
  #include <fcntl.h>
  #include <unistd.h>
  #include <sys/time.h>
  #include <sys/select.h>
#endif  // __linux__

namespace RoverLib2
{
#if defined(__linux__)
bool hasRTSPStream(const std::string& url_, size_t timeoutMs_)
{
    // --- 1) Extract host, port and path from the full RTSP URL ---
    auto start = url_.find("rtsp://");
    if (start == std::string::npos) return false;
    start += 7;

    // strip off "rtsp://…"
    std::string tail = url_.substr(start);

    // drop any credentials "user:pass@"
    auto at = tail.find('@');
    if (at != std::string::npos)
        tail = tail.substr(at + 1);

    // split into "[host[:port]]" and "/path…"
    std::string hostport, path;
    auto slash = tail.find('/');
    if (slash == std::string::npos) {
        hostport = tail;
        path     = "/";
    } else {
        hostport = tail.substr(0, slash);
        path     = tail.substr(slash);
    }

    // split host vs port
    std::string host;
    uint16_t    port = 554;  // default RTSP
    auto colon = hostport.rfind(':');
    if (colon != std::string::npos) {
        host = hostport.substr(0, colon);
        port = static_cast<uint16_t>(std::stoi(hostport.substr(colon + 1)));
    } else {
        host = hostport;
    }
    if (host.empty()) return false;

    // --- 2) Build sockaddr_in (reuse your DNS / inet_addr logic) ---
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);

    if (inet_addr(host.c_str()) == INADDR_NONE) {
        addrinfo hints{}, *res = nullptr;
        hints.ai_family = AF_INET;
        if (getaddrinfo(host.c_str(), nullptr, &hints, &res) != 0)
            return false;
        std::memcpy(&addr.sin_addr,
                    &reinterpret_cast<sockaddr_in*>(res->ai_addr)->sin_addr,
                    sizeof(in_addr));
        freeaddrinfo(res);
    } else {
        addr.sin_addr.s_addr = inet_addr(host.c_str());
    }

    // --- 3) Open non-blocking socket + connect with timeout ---
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return false;

    int flags = fcntl(sock, F_GETFL, 0);
    if (flags < 0) { close(sock); return false; }
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    if (connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0
        && errno != EINPROGRESS)
    {
        close(sock);
        return false;
    }

    // wait for connect→write readiness
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(sock, &wfds);
    timeval tv{ static_cast<long>(timeoutMs_ / 1000),
               static_cast<long>((timeoutMs_ % 1000) * 1000) };
    if (select(sock + 1, nullptr, &wfds, nullptr, &tv) <= 0) {
        close(sock);
        return false;
    }

    // check for socket error
    int sockErr = 0;
    socklen_t len = sizeof(sockErr);
    getsockopt(sock, SOL_SOCKET, SO_ERROR, &sockErr, &len);
    if (sockErr != 0) {
        close(sock);
        return false;
    }

    // --- 4) Send RTSP DESCRIBE request ---
    std::string req =
        "DESCRIBE " + url_ + " RTSP/1.0\r\n"
        "CSeq: 1\r\n"
        "Accept: application/sdp\r\n"
        "\r\n";
    ::write(sock, req.data(), req.size());

    // --- 5) Wait for and read the response status line ---
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(sock, &rfds);
    // reuse same timeout if you like, or reset tv...
    if (select(sock + 1, &rfds, nullptr, nullptr, &tv) <= 0) {
        close(sock);
        return false;
    }

    char buf[512];
    int  n = ::read(sock, buf, sizeof(buf)-1);
    close(sock);
    if (n <= 0) return false;
    buf[n] = '\0';

    // grab up to the first "\r\n"
    auto eol = std::strstr(buf, "\r\n");
    if (!eol)      return false;
    std::string statusLine(buf, eol - buf);

    // --- 6) Check for "200 OK" ---
    return statusLine.find("200 OK") != std::string::npos;
}
#endif  // __linux__
}  // namespace RoverLib2
