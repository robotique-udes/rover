#ifndef ANTENNA_COMMAND_HPP
#define ANTENNA_COMMAND_HPP
#include <cstdint>
#include <string>

enum class eAntennaCode : uint8_t
{
    SUCCESS = 0,
    FAILURE_DEVICE_OFFLINE,
    FAILURE_SESSION_EXPIRED,
    FAILURE_PARSING_ERROR,
    FAILURE_ON_COOLDOWN,
    FAILURE_UNKNOWN
};

enum class eHttpStatus : uint16_t
{
    OFFLINE = 0,
    OK = 200,
    UNAUTHORIZED = 401,
    FORBIDDEN = 403
};

/**
 * @brief Interface for Antenna commands
 *
 */
class AntennaCommand
{
  public:
    explicit AntennaCommand(const std::string& apiUrl_):
        _apiUrl(apiUrl_)
    {
    }
    virtual ~AntennaCommand() = default;
    virtual eAntennaCode execute(void) = 0;

  protected:
    std::string getApiUrl(void) const
    {
        return _apiUrl;
    }

  private:
    const std::string _apiUrl;
};

#endif  // ANTENNA_COMMAND_HPP