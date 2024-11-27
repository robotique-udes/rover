#include "QTmpFolderManager.hpp"
#include <QDir>
#include <QUuid>

#include <QString>

#include "rclcpp/rclcpp.hpp"

QTmpFolderManager::QTmpFolderManager()
{
    std::string tempBasePath = QDir::tempPath().toStdString() + "/rover-gui-tmp-folder";

    QDir dir(QString::fromStdString(tempBasePath));
    if (dir.exists())
    {
        dir.removeRecursively();
    }

    std::string sessionFolderName = "/" + QUuid::createUuid().toString().toStdString();
    _tempFolderPath = (tempBasePath + sessionFolderName);

    _valid = createDirectory(_tempFolderPath);
}

QTmpFolderManager& QTmpFolderManager::getInstance()
{
    static QTmpFolderManager instance;
    return instance;
}

bool QTmpFolderManager::getTmpFolderPath(OUT std::string& path_) const
{
    if (_valid)
    {
        path_ = _tempFolderPath;
    }

    return _valid;
}

bool QTmpFolderManager::getUniqueTmpFolderPath(OUT std::string& path_) const
{
    bool success = this->getTmpFolderPath(path_);

    if (success)
    {
        path_ += "/" + QUuid::createUuid().toString().toStdString();
        success = createDirectory(path_);
    }

    return success;
}

bool QTmpFolderManager::createDirectory(const std::string& path_) const
{
    bool success = true;
    if (!QDir().mkpath(QString::fromStdString(path_)))
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"), "Failed to create directory: " << path_);
        success = false;
    }

    return success;
}
