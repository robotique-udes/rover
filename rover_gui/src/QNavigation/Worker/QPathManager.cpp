#include "QPathManager.hpp"

#include <filesystem>
#include <sstream>
#include <iomanip>
#include <QVariantMap>

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

void QPathManager::initializeCSVFile(QVariantList& oldPath_)
{
    std::string currentFilePath = _sessionFolderPath + POSITION_FILE_PATH;

    if (!std::filesystem::exists(currentFilePath))
    {
        std::string lastSessionFolderPath = this->findLastSessionFolder();
        std::string lastFilePath = lastSessionFolderPath + POSITION_FILE_PATH;
        if (!std::filesystem::exists(lastFilePath))
        {
            RCLCPP_WARN(rclcpp::get_logger("GUI"), "Unable to load Position.csv. File missing or invalid.");
            return;
        }
        std::filesystem::copy_file(lastFilePath, currentFilePath);
    }

    this->addTask(
        [this, currentFilePath, &oldPath_]
        {
            RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Starting to read at path: %s", currentFilePath.c_str());
            this->readFromCSV(currentFilePath, oldPath_);
        });
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
        csv_file << std::fixed << std::setprecision(8) << latitude_ << "," << longitude_ << std::endl;
        RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Appending file at path: %s", filePath.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Unable to open. Lost coordinates");
    }
}

void QPathManager::readFromCSV(std::string filePath_, QVariantList& oldPath_)
{
    std::ifstream file(filePath_);
    if (!file.is_open())
    {
        return;
    }
    std::string line;

    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        std::string latStr, lonStr;
        if (std::getline(ss, latStr, ',') && std::getline(ss, lonStr, ','))
        {
            QVariantMap point;
            point["latitude"] = QString::fromStdString(latStr).toDouble();
            point["longitude"] = QString::fromStdString(lonStr).toDouble();
            oldPath_.append(point);
        }
    }
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