#ifndef __ROVUS_EXCEPTIONS_H__
#define __ROVUS_EXCEPTIONS_H__

#include <stdexcept>
#include "macros.h"

// =============================================================================
// This header file is there to act as a library to define custom exception used
//  nodes from other rovus ros pkgs
// =============================================================================

class ExeptBadLaunchParameters : public std::exception
{
public:
    ExeptBadLaunchParameters(const char *message)
        : errorMessage(std::string("Bad Launch Parameters: " + std::string(message))) {}

    const char *what() const noexcept override
    {
        return errorMessage.c_str();
    }

private:
    std::string errorMessage;
};

#endif //__ROVUS_EXCEPTION_H__
