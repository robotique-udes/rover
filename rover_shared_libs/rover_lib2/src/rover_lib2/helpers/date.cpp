#include "date.hpp"

#if defined(__linux__)

#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>

std::string Date::getCurrentTime(void)
{
    std::stringstream current_time_output;

    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);  // convert to real time
    std::tm tm_now = *std::localtime(&now_time);                       // convert to calendar time
    current_time_output << std::put_time(&tm_now, "%FT%T");            // ISO 8601 format

    return current_time_output.str();
}

#endif  // defined(__linux__)