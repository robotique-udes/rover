#include "QSshWorker.hpp"

#include <QDesktopServices>
#include <QFile>
#include <QUrl>

#include <fcntl.h>
#include <rclcpp/rclcpp.hpp>

#include "Global/Helpers/QHelpers.hpp"
#include "LibSshSupportModule.hpp"
#include "QSshFileExplorer/QDownloadedFileManager.hpp"

#warning TODO: Documentation in global helpers and support module
#warning TODO: No connection dialog
#warning TODO: Dialog helper keyboard shortcuts
#warning TODO: Music folder (simlink?)

std::mutex QSshWorker::_libSshMutex;

QSshWorker::QSshWorker(bool start_, QObject* parent_): QWorker(start_, parent_) {}

QSshWorker::~QSshWorker()
{
    this->finish();
}

void QSshWorker::refreshStructure(std::string username_, std::string hostname_, std::string path_)
{
    this->addTask([username = std::move(username_), hostname = std::move(hostname_), path = std::move(path_), this](void)
                  { this->refreshStructureInternal(username, hostname, path); });
}

void QSshWorker::openFile(IN const std::string& rUsername_, IN const std::string& rHostname_, IN const std::string& rfilePath_)
{
    this->addTask([username = std::move(rUsername_), hostname = std::move(rHostname_), path = std::move(rfilePath_), this](void)
                  { this->downloadFileInternal(username, hostname, path); });

    this->addTask([path = QHelper::getFileNameFromPath(rfilePath_), this](void) { this->openLocalFile(path); });
}

std::vector<QFileItem> QSshWorker::getFileStructure(void)
{
    std::lock_guard<std::mutex> lock(_filesMutex);
    return _files;
}

void QSshWorker::refreshStructureInternal(std::string username_, std::string hostname_, std::string path_)
{
    bool success = true;
    ssh_session pSshSession = nullptr;
    sftp_session pSftpSession = nullptr;
    sftp_dir pSftpDir = nullptr;

    std::unique_lock<std::mutex> lockLibSsh(_libSshMutex);
    if (!LibSshSupportModule::getSshSession(username_, hostname_, pSshSession) || !pSshSession)
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"), "Error creating SSH session, device ssh service might not be running");
        success = false;
    }

    if (success && (!pSshSession || !LibSshSupportModule::getSftpSessions(pSshSession, pSftpSession) || !pSftpSession))
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"), "Error creating SFTP session: " << ssh_get_error(pSshSession));
        success = false;
    }

    if (success && sftp_init(pSftpSession) != SSH_OK)
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"), "Error initializing SFTP session: " << ssh_get_error(pSftpSession));
        success = false;
    }

    if (success)
    {
        pSftpDir = sftp_opendir(pSftpSession, path_.c_str());
        if (!pSftpDir)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("GUI"),
                                "Error opening directory \"" << path_.c_str() << "\" " << sftp_get_error(pSftpSession));
            success = false;
        }
    }

    if (success)
    {
        sftp_attributes pSftpAttribute = nullptr;
        std::vector<sftp_attributes> filesAttribute;
        std::vector<sftp_attributes> folderAttribute;
        std::vector<sftp_attributes> otherAttribute;

        while ((pSftpAttribute = sftp_readdir(pSftpSession, pSftpDir)) && pSftpAttribute->name && pSftpAttribute->permissions)
        {
            if (pSftpAttribute && (pSftpAttribute->permissions & SSH_S_IFDIR))
            {
                if (std::string(pSftpAttribute->name) != ".")
                {
                    folderAttribute.push_back(pSftpAttribute);
                }
                else
                {
                    sftp_attributes_free(pSftpAttribute);
                }
            }
            else if (pSftpAttribute && !(pSftpAttribute->permissions & SSH_S_IFDIR))
            {
                filesAttribute.push_back(pSftpAttribute);
            }
            else if (pSftpAttribute)
            {
                sftp_attributes_free(pSftpAttribute);
            }

            pSftpAttribute = nullptr;
        }

        LibSshSupportModule::sortSftpAttributeVector(folderAttribute);
        LibSshSupportModule::sortSftpAttributeVector(filesAttribute);
        LibSshSupportModule::sortSftpAttributeVector(otherAttribute);

        {
            std::unique_lock<std::mutex> lock(_filesMutex);
            _files.clear();

            for (auto& it : folderAttribute)
            {
                _files.push_back(QFileItem(it->name, "", LibSshSupportModule::unixTimeToString(it->mtime)));
                sftp_attributes_free(it);
            }
            for (auto& it : filesAttribute)
            {
                _files.push_back(
                    QFileItem(it->name, QHelper::getFileExtension(it->name), LibSshSupportModule::unixTimeToString(it->mtime)));
                sftp_attributes_free(it);
            }
            for (auto& it : otherAttribute)
            {
                _files.push_back(QFileItem(it->name, "*", LibSshSupportModule::unixTimeToString(it->mtime)));
                sftp_attributes_free(it);
            }
        }

        emit this->newStructureReady();
    }

    if (pSftpDir)
    {
        sftp_closedir(pSftpDir);
        pSftpDir = nullptr;
    }

    if (pSftpSession)
    {
        sftp_free(pSftpSession);
        pSftpSession = nullptr;
    }

    if (pSshSession)
    {
        ssh_disconnect(pSshSession);
        ssh_free(pSshSession);
        pSshSession = nullptr;
    }
}

void QSshWorker::downloadFileInternal(IN const std::string& rUsername_,
                                      IN const std::string& rHostname_,
                                      IN const std::string& rRemoteFilePath_)
{
    bool success = true;

    ssh_session pSSHSession = nullptr;
    sftp_session pSftpSession = nullptr;
    sftp_file pfile = nullptr;

    std::unique_lock<std::mutex> lockLibSsh(_libSshMutex);
    if (!LibSshSupportModule::getSshSession(rUsername_, rHostname_, pSSHSession) || !pSSHSession)
    {
        success = false;
    }

    if (success && (!LibSshSupportModule::getSftpSessions(pSSHSession, pSftpSession) || !pSftpSession))
    {
        success = false;
    }

    uint64_t fileSize = 0u;
    success = LibSshSupportModule::getSftpFileSize(pSftpSession, rRemoteFilePath_, fileSize);

    switch (QDownloadedFileManager::getInstance().alreadyDownloaded(QHelper::getFileNameFromPath(rRemoteFilePath_), fileSize))
    {
        case QDownloadedFileManager::eDownloadState::ALREADY_DOWNLOADED_OK: success = false; break;
        case QDownloadedFileManager::eDownloadState::ALREADY_DOWNLOADED_SIZE_MISSMATCH:
        {
#warning TODO: Prompt user for action
        }
        case QDownloadedFileManager::eDownloadState::NOT_DOWNLOADED: success = true; break;
    }

    if (success)
    {
        pfile = sftp_open(pSftpSession, rRemoteFilePath_.c_str(), O_RDONLY, 0);
        if (!pfile)
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"),
                               "Error opening file: " << rUsername_ << "@" << rHostname_ << ":" << rRemoteFilePath_ << " ("
                                                      << ssh_get_error(pSSHSession) << ")");
            success = false;
        }
    }

    std::string tmpFolderPath;
    if (success && !QTmpFolderManager::getInstance().getTmpFolderPath(tmpFolderPath))
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("GUI"),
                            "Couldn't create tmp folder for this gui session, expect some undefined behaviors");
        success = false;
    }

    std::string fileName = QHelper::getFileNameFromPath(rRemoteFilePath_);
    QFile localFile(QString::fromStdString(tmpFolderPath + "/" + fileName));
    if (success)
    {
        if (!localFile.open(QIODevice::WriteOnly))
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"), "Couldn't open local file for writting");
            success = false;
        }
    }

    if (success)
    {
        uint8_t buffer[FILE_DOWNLOAD_BUFFER_SIZE] = {0};
        ssize_t nbytes = 0;
        uint64_t totalBytesRead = 0;

        uint32_t progress = 0u;
        while ((nbytes = sftp_read(pfile, buffer, sizeof(buffer))) > 0)
        {
            localFile.write(reinterpret_cast<const char*>(buffer), nbytes);

            totalBytesRead += static_cast<uint64_t>(CONSTRAIN(nbytes, 0, sizeof(buffer)));
            progress = static_cast<uint64_t>(static_cast<float>(totalBytesRead) / static_cast<float>(fileSize) * 1'000'000.0f);
            if (totalBytesRead != 0 && fileSize != 0 && (uint64_t)progress % 100'000 == 0)
            {
                auto tmp_clock = rclcpp::Clock();
                RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger("GUI"),
                                            tmp_clock,
                                            1000,
                                            "[" << progress / 10'000 << " %]"
                                                << " Download of " << rRemoteFilePath_);
            }
        }

        if (nbytes >= 0)
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("GUI"), "Finished download of " << rRemoteFilePath_ << " successfully");
            QDownloadedFileManager::getInstance().addFileToList(QHelper::getFileNameFromPath(rRemoteFilePath_), fileSize);
        }
        else
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"),
                               "Error (" << nbytes << ") "
                                         << "while transfering file");
            success = false;
        }
    }

    if (pfile)
    {
        sftp_close(pfile);
        pfile = nullptr;
    }
    if (pSftpSession)
    {
        sftp_free(pSftpSession);
        pSftpSession = nullptr;
    }
    if (pSSHSession)
    {
        ssh_disconnect(pSSHSession);
        ssh_free(pSSHSession);
        pSSHSession = nullptr;
    }

    if (!success)
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"),
                           "File transfer of " << rUsername_ << "@" << rHostname_ << ":" << rRemoteFilePath_
                                               << " was unsuccessful.");
    }
}

void QSshWorker::openLocalFile(IN const std::string& fileName_)
{
    if (QDownloadedFileManager::getInstance().alreadyDownloaded(fileName_, 0u)
        != QDownloadedFileManager::eDownloadState::NOT_DOWNLOADED)
    {
        std::string tmpFolderPath;
        if (QTmpFolderManager::getInstance().getTmpFolderPath(tmpFolderPath))
        {
            QDesktopServices::openUrl(QUrl::fromLocalFile((tmpFolderPath + "/" + fileName_).c_str()));
        }
    }
    else
    {
#warning TODO: Print fail
    }
}
