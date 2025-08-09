#include "QPathManager.hpp"

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
    std::string filePath = _sessionFolderPath + POSITION_FILE_PATH;
    std::ofstream csv_file(filePath);

    if (csv_file.is_open())
    {
        csv_file << "latitude,longitude" << std::endl;
        RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Creating and writing header to file at path: %s", filePath.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Unable to open Position csv.");
        return;
    }
}

void QPathManager::writePosToCSV(double latitude_, double longitude_)
{
    this->addTask(
        [this, latitude_, longitude_]
        (void)
        {
            this->writePosToCSVInternal(latitude_, longitude_);
        });
}

void QPathManager::writePosToCSVInternal(double latitude_, double longitude_)
{
    std::string filePath = _sessionFolderPath + POSITION_FILE_PATH;
    RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Opening file at path: %s", filePath.c_str());
    std::ofstream csv_file(filePath, std::ios_base::app);

    if (csv_file.is_open())
    {
        csv_file << latitude_ << "," << longitude_ << std::endl;
        RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Appending file at path: %s", filePath.c_str());
        return;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Unable to open. Lost coordinates");
        return;
    }

    emit this->onCSVReady();
}