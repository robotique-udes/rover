#ifndef ROVER_LIB2_HELPERS_DATE_HPP
#define ROVER_LIB2_HELPERS_DATE_HPP

#include <string>

namespace Date
{

#if defined(__linux__)

    /**
     * @brief Get the current time in ISO 8601 format
     */
    std::string getCurrentTime(void);

#endif  // defined(__linux__)
}  // namespace Date

#endif  // ROVER_LIB2_HELPERS_DATE_HPP
