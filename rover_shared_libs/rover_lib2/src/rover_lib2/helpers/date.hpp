#ifndef DATE_HPP
#define DATE_HPP

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

#endif  // DATE_HPP