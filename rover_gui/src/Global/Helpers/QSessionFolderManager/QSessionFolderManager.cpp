#include "QSessionFolderManager.hpp"
#include <QDir>
#include "rclcpp/rclcpp.hpp"
#include <rover_lib2/helpers/date.hpp>

QSessionFolderManager::QSessionFolderManager()
{
    std::string currentTime = Date::getCurrentTime();
    _sessionFolderPath = "/rover-autogen/rover-session/" + currentTime;
}

QSessionFolderManager& QSessionFolderManager::getInstance(void)
{
    static QSessionFolderManager instance;
    return instance;
}

std::optional<std::string> QSessionFolderManager::getSessionFolderPath(void) const
{
    return _sessionFolderPath;
}