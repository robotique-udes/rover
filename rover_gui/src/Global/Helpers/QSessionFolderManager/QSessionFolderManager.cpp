#include "QSessionFolderManager.hpp"
#include <QDir>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include "rclcpp/rclcpp.hpp"

QSessionFolderManager::QSessionFolderManager()
{
    std::string currentTime = this->getCurrentTime();
    _sessionFolderPath = "/rover-autogen/rover-session/" + currentTime;
}

QSessionFolderManager& QSessionFolderManager::getInstance(void)
{
    static QSessionFolderManager instance;
    return instance;
}

std::string QSessionFolderManager::getCurrentTime(void)
{
    std::stringstream current_time_output;

    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);  // convert to real time
    std::tm tm_now = *std::localtime(&now_time);                       // convert to calendar time
    current_time_output << std::put_time(&tm_now, "%FT%T");            // ISO 8601 format

    return current_time_output.str();
}

std::optional<std::string> QSessionFolderManager::getSessionFolderPath(void) const
{
    return _sessionFolderPath;
}