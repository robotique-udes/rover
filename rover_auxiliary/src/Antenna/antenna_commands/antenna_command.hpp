#ifndef ANTENNA_COMMAND_HPP
#define ANTENNA_COMMAND_HPP
#include <cpr/cpr.h>
#include "antenna_msg.hpp"

struct sCommandResult
{
    bool success = false;
    std::string error;
    uint16_t httpStatus = 0;
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
    AntennaCommand(const std::string& apiUrl_):
        _apiUrl(apiUrl_)
    {
    }
    virtual ~AntennaCommand() = default;
    virtual sCommandResult execute(std::shared_ptr<cpr::Session> session_, sAntennaMsg& msg_) = 0;

  protected:
    const std::string& getApiUrl(void) const
    {
        return _apiUrl;
    }

  private:
    std::string _apiUrl;
};

#endif  // ANTENNA_COMMAND_HPP