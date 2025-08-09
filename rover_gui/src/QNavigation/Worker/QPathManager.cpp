#include "QPathManager.hpp"

#include <filesystem>

QPathManager::QPathManager(bool start_, QObject* parent_):
    QWorker(start_, parent_)
{
}

void QPathManager::setSessionFolderPath(std::string sessionFolderPath_)
{
    if (!sessionFolderPath_.empty())
    {
        _sessionFolderPath = sessionFolderPath_;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Session folder path empty. Couldn't assign it");
    }
}

void QPathManager::initializeCSVFile(void)
{
    std::string currentFilePath = _sessionFolderPath + POSITION_FILE_PATH;
    
    if (!std::filesystem::exists(currentFilePath))
    {
        std::string lastSessionFolderPath = this->findLastSessionFolder();
        std::string lastFilePath = lastSessionFolderPath + POSITION_FILE_PATH;
        if (!std::filesystem::exists(lastFilePath))
        {
            RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Unable to load waypoint from JSON. File missing or invalid.");
            return;
        }
        std::filesystem::copy_file(lastFilePath, currentFilePath);
    }

}

void QPathManager::writePosToCSV(double latitude_, double longitude_)
{
    this->addTask(
        [this, latitude_, longitude_](void)
        {
            this->writePosToCSVInternal(latitude_, longitude_);
        });
}

void QPathManager::writePosToCSVInternal(double latitude_, double longitude_)
{
    std::string filePath = _sessionFolderPath + POSITION_FILE_PATH;
    std::ofstream csv_file(filePath, std::ios_base::app);

    if (csv_file.is_open())
    {
        csv_file << latitude_ << "," << longitude_ << std::endl;
        RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Appending file at path: %s", filePath.c_str());
        emit this->onCSVReady();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Unable to open. Lost coordinates");
    }
}

void QPathManager::readFromCSV(std::string filePath_)
{
    std::ifstream csv_file(filePath_);

    csv_file.close();
}

std::string QPathManager::findLastSessionFolder(void)
{
    std::filesystem::path currentPath(_sessionFolderPath);
    std::filesystem::path sessionBasePath = currentPath.parent_path().parent_path();

    if (!std::filesystem::exists(sessionBasePath))
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Session base path doesn't exist: %s", sessionBasePath.c_str());
        return "";
    }

    std::vector<std::string> sessionFolders;
    for (const auto& entry : std::filesystem::directory_iterator(sessionBasePath))
    {
        if (entry.is_directory())
        {
            sessionFolders.push_back(entry.path().string());
        }
    }

    std::sort(sessionFolders.begin(), sessionFolders.end(), std::greater<std::string>());

    if (sessionFolders.size() > 1)
    {
        std::string lastSessionFolderPath = sessionFolders[1] + NAVIGATION_PATH;
        RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Found latest session folder: %s", lastSessionFolderPath.c_str());
        return lastSessionFolderPath;
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Couldn't find last session");
        return "";
    }
}