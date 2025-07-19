#ifndef ANTENNA_COMMAND_HPP
#define ANTENNA_COMMAND_HPP
#include <cpr/cpr>
#include "antenna_msg.hpp"

/**
 * @brief Interface for Antenna commands
 *
 */
class AntennaCommand
{
  public:
    virtual ~AntennaCommand() = default;
    virtual bool execute(std::shared_ptr<cpr::Session> session_, sAntennaMsg& msg_) = 0;
};

#endif  // ANTENNA_COMMAND_HPP