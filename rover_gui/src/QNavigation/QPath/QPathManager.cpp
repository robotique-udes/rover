#include "QPathManager.hpp"

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

void QPathManager::initializeCSVFile()
{
    std::string filePath = _sessionFolderPath + POSITION_FILE_PATH;
    std::ofstream csv_file(filePath);

    if (csv_file.is_open())
    {
        csv_file << "latitude,longitude" << std::endl;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Unable to open Position csv.");
        return;
    }
}

void QPathManager::writePosToCSV(double latitude_, double longitude_)
{
    std::string filePath = _sessionFolderPath + POSITION_FILE_PATH;
    std::ofstream csv_file(filePath, std::ios_base::app);

    if (csv_file.is_open())
    {
        csv_file << latitude_ << "," << longitude_ << std::endl;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Unable to open. Lost coordinates");
        return;
    }
}