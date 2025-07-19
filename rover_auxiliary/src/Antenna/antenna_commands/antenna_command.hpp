#ifndef ANTENNA_COMMAND_HPP
#define ANTENNA_COMMAND_HPP
#include <cpr/cpr.h>
#include "antenna_msg.hpp"

struct sCommandResult
{
    bool success = false;
    std::string error;
    operator bool() const
    {
        return success;
    }
};

enum class eHttpStatus : uint16_t
{
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
  protected:
    std::string _baseURL;

  public:
    AntennaCommand(const std::string& baseURL_):
        _baseURL(baseURL_)
    {
    }
    virtual ~AntennaCommand() = default;
    virtual sCommandResult execute(std::shared_ptr<cpr::Session> session_, sAntennaMsg& msg_) = 0;
};

#endif  // ANTENNA_COMMAND_HPP