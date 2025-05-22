#include "QSessionFolderManager.hpp"
#include <QDir>
#include "rclcpp/rclcpp.hpp"

QSessionFolderManager::QSessionFolderManager()
{
    _valid = false;
    const char* home = std::getenv("HOME");
    std::string homeStr;
    if (home)
    {
        homeStr = home;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Unable to create session folder, home path was not defined");
        return;
    }

    std::string currentTime = this->getCurrentTime();
    _sessionFolderPath = homeStr + "/Rover_session/" + currentTime;

    _valid = this->createDirectory(_sessionFolderPath);
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

bool QSessionFolderManager::getSessionFolderPath(OUT std::string& path_) const
{
    if (_valid)
    {
        path_ = _sessionFolderPath;
    }

    return _valid;
}

bool QSessionFolderManager::createDirectory(const std::string& path_) const
{
    bool success = true;
    if (!QDir().mkpath(QString::fromStdString(path_)))
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"), "Failed to create directory: " << path_);
        success = false;
        return success;
    }

    for (const char* subdirectory : SUBDIRECTORIES)
    {
        std::string sub = subdirectory;
        if (!QDir().mkpath(QString::fromStdString(path_ + sub)))
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"), "Failed to create subdirectory: " << sub);
            success = false;
            return success;
        }
    }

    return success;
}