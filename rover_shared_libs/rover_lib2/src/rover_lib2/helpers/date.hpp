#ifndef DATE_HPP
#define DATE_HPP

namespace Date
{

#if defined(__linux__)

#include <string>

std::string getCurrentTime(void);

#endif //defined(__linux__)
} //namespace Date

#endif //DATE_HPP