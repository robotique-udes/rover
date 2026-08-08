#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#if defined(__linux__) && defined(ROS)

#include "rover_lib2/helpers/constants.hpp"
#include "rover_lib2/helpers/log.hpp"

#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <optional>
#include <string>

enum class eBaudRate : speed_t
{
    B_57600 = B57600,
    B_115200 = B115200,
    B_230400 = B230400,
    B_460800 = B460800,
    B_500000 = B500000,
    B_576000 = B576000,
    B_921600 = B921600,
    B_1000000 = B1000000,
    B_1152000 = B1152000,
    B_1500000 = B1500000,
    B_2000000 = B2000000,
    B_2500000 = B2500000,
    B_3000000 = B3000000,
    B_3500000 = B3500000,
    B_4000000 = B4000000
};

enum class eDataPerPacket : tcflag_t
{
    FIVE_BITS = CS5,
    SIX_BITS = CS6,
    SEVEN_BITS = CS7,
    EIGHT_BITS = CS8
};

enum class eState : uint8_t
{
    INACTIVE,
    ACTIVE
};

DEFINE_LOG_NODE(SerialCom, Logger::eNodeState::ON);

/**
 * @brief RAII wrapper around a POSIX serial port file descriptor.
 *
 * Provides open/configure/read/write/close lifecycle management for a
 * serial device using POSIX termios. Not copyable or movable, as it
 * owns a single file descriptor for its lifetime.
 *
 * @note POSIX systems only.
 * @warning Not thread-safe. Concurrent calls to serialRead() / serialWrite()
 *          / reconnect() from multiple threads require external synchronization.
 */
class SerialCom
{
  public:
    SerialCom(std::string path_,
              eBaudRate baudRate_ = eBaudRate::B_115200,
              eDataPerPacket char_ = eDataPerPacket::EIGHT_BITS,
              tcflag_t cflags_ = CREAD | CLOCAL,
              uint8_t minChar_ = 0,
              uint8_t timeout_ = 1);
    SerialCom(const SerialCom&) = delete;
    SerialCom& operator=(const SerialCom&) = delete;
    SerialCom(SerialCom&&) = delete;
    SerialCom& operator=(SerialCom&&) = delete;
    ~SerialCom();
    bool serialWrite(const std::string& cmd_);

    /**
     * @brief Reads available bytes from the serial port.
     *
     * @return Raw bytes read, or std::nullopt on inactive port / read failure.
     */
    std::optional<std::string> serialRead();
    eState getState() const;
    bool reconnect();
    void flushInput();

  private:
    static constexpr uint16_t READING_BUFFER = 256;
    static constexpr char FRAME_TERMINATOR = '\r';
    static constexpr uint16_t READ_TIMEOUT_MS = 200;
    static constexpr uint16_t MAX_RX_SIZE = 512;
    static constexpr std::chrono::milliseconds LOGGER_THROTTLE_MS = std::chrono::milliseconds(20'000);

    bool serialConfig();
    int _fileDesc;
    eState _state;
    std::string _path;
    eBaudRate _baudRate;
    eDataPerPacket _char;
    tcflag_t _cflags;
    uint8_t _minChar;
    uint8_t _timeout;
    std::string _rxBuffer;
    std::chrono::time_point<std::chrono::steady_clock> _lastLogDC{};
};

#else
#error "SerialCom requires Linux (POSIX termios). Not available on this target."
#endif  // __linux__
#endif