#include "QSshWorker.hpp"

#include <QDesktopServices>
#include <QFile>
#include <QUrl>

#include <fcntl.h>
#include <rclcpp/rclcpp.hpp>

#include "Global/Helpers/QHelpers.hpp"
#include "LibSshSupportModule.hpp"
#include "QSshFileExplorer/QDownloadedFileManager.hpp"

std::mutex QSshWorker::_libSshMutex;

QSshWorker::QSshWorker(bool start_, QObject* parent_):
    QWorker(start_, parent_)
{
    connect(this,
            &QWorker::allTasksDone,
            this,
            [this]()
            {
                emit this->newProgressBarUpdate("", 100.0f);
            });
}

QSshWorker::~QSshWorker()
{
    this->finish();
}

void QSshWorker::refreshStructure(const std::string& username_,
                                  const std::string& hostname_,
                                  const std::string& oldPath_,
                                  const std::string& newPath_)
{
    this->addTask(
        [username = std::move(username_),
         hostname = std::move(hostname_),
         oldPath = std::move(oldPath_),
         newPath = std::move(newPath_),
         this](void)
        {
            this->refreshStructureInternal(username, hostname, oldPath, newPath);
        });

    emit this->newProgressBarUpdate(std::string("Getting items for " + newPath_), 0.0f);
}

void QSshWorker::openFile(const std::string& rUsername_, const std::string& rHostname_, const std::string& rfilePath_)
{
    this->addTask(
        [username = std::move(rUsername_), hostname = std::move(rHostname_), path = std::move(rfilePath_), this](void)
        {
            this->downloadFileInternal(username, hostname, path);
        });
    this->addTask(
        [path = QHelper::getFileNameFromPath(rfilePath_), this](void)
        {
            this->openLocalFile(path);
        });

    emit this->newProgressBarUpdate("", 0.0f);
}

void QSshWorker::transferFile(const std::string& fileName_,
                              const std::string& ownerUsername_,
                              const std::string& ownerHostname_,
                              const std::string& ownerFolderPath_,
                              const std::string& receiverUsername_,
                              const std::string& receiverHostname_,
                              const std::string& receiverFolderPath_)
{
    this->addTask(
        [username = std::move(ownerUsername_),
         hostname = std::move(ownerHostname_),
         filePath = std::move(ownerFolderPath_ + "/" + fileName_),
         this](void)
        {
            this->downloadFileInternal(username, hostname, filePath);
        });

    this->addTask(
        [username = std::move(receiverUsername_),
         hostname = std::move(receiverHostname_),
         fileName = std::move(fileName_),
         folderPath = std::move(receiverFolderPath_),
         this](void)
        {
            this->uploadFileInternal(username, hostname, fileName, folderPath);
        });
}

std::vector<QFileItem> QSshWorker::getFileStructure(void)
{
    std::lock_guard<std::mutex> lock(_filesMutex);
    return _files;
}

const std::string& QSshWorker::getPathStructure(void)
{
    return _filesPath;
}

void QSshWorker::refreshStructureInternal(const std::string& username_,
                                          const std::string& hostname_,
                                          const std::string& oldPath_,
                                          const std::string& newPath_)
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
        pSftpDir = sftp_opendir(pSftpSession, newPath_.c_str());
        if (!pSftpDir)
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"),
                               "Error opening directory \"" << newPath_.c_str() << "\" " << sftp_get_error(pSftpSession));
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

    if (success)
    {
        _filesPath = newPath_;
    }
    else
    {
        _filesPath = oldPath_;
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
        case QDownloadedFileManager::eDownloadState::ALREADY_DOWNLOADED_OK:
            success = false;
            break;
        case QDownloadedFileManager::eDownloadState::ALREADY_DOWNLOADED_SIZE_MISSMATCH:
        {
            QMessageBox::StandardButton userSelection = QMessageBox::StandardButton::No;
            userSelection = QHelper::QPopUp::sendQuestionPopUp(
                "File conclicts warning",
                "A file of with this name has already been cached, are you sure you want to <b>overide</b> it?",
                QMessageBox::StandardButton::Yes | QMessageBox::StandardButton::No);

            success = userSelection == QMessageBox::StandardButton::Yes ? true : false;
            break;
        }
        case QDownloadedFileManager::eDownloadState::NOT_DOWNLOADED:
            success = true;
            break;
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
    if (success && !localFile.open(QIODevice::WriteOnly))
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"), "Couldn't open local file for writting");
        success = false;
    }

    if (success)
    {
        uint8_t buffer[FILE_TRANSFER_BUFFER_SIZE] = {0};
        ssize_t nbytes = 0;
        uint64_t totalBytesRead = 0;

        float progress = 0.0f;
        while (!_cancelCurrentTasksFlag.load() && (nbytes = sftp_read(pfile, buffer, sizeof(buffer))) > 0)
        {
            localFile.write(reinterpret_cast<const char*>(buffer), nbytes);

            totalBytesRead += static_cast<uint64_t>(CONSTRAIN(nbytes, 0, sizeof(buffer)));
            progress = 100.0f * static_cast<float>(totalBytesRead) / static_cast<float>(fileSize);
            if (totalBytesRead != 0 && fileSize != 0)
            {
                auto tmp_clock = rclcpp::Clock();
                RCLCPP_INFO_THROTTLE(rclcpp::get_logger("GUI"),
                                     tmp_clock,
                                     1'000,
                                     "[ %f %%] Download of %s",
                                     progress,
                                     rRemoteFilePath_.c_str());
                emit this->newProgressBarUpdate(std::string(" Downloading ") + rRemoteFilePath_ + "...", progress);
            }
        }

        if (!_cancelCurrentTasksFlag.load() && nbytes >= 0)
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("GUI"), "Finished download of " << rRemoteFilePath_ << " successfully");
            QDownloadedFileManager::getInstance().addFileToList(QHelper::getFileNameFromPath(rRemoteFilePath_), fileSize);
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

    if (!_cancelCurrentTasksFlag.load() && !success)
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"),
                           "File transfer of " << rUsername_ << "@" << rHostname_ << ":" << rRemoteFilePath_
                                               << " was unsuccessful.");
    }
    else if (_cancelCurrentTasksFlag.load())
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"),
                           "File transfer of " << rUsername_ << "@" << rHostname_ << ":" << rRemoteFilePath_
                                               << " was canceled by the user.");
    }
}

void QSshWorker::uploadFileInternal(IN const std::string& rUsername_,
                                    IN const std::string& rHostname_,
                                    IN const std::string& rFileName_,
                                    IN const std::string& rRemoteFolderPath_)
{
    bool success = true;

    ssh_session pSSHSession = nullptr;
    sftp_session pSftpSession = nullptr;
    sftp_file pfile = nullptr;
    std::unique_ptr<QFile> localFile = nullptr;

    std::unique_lock<std::mutex> lockLibSsh(_libSshMutex);
    if (!LibSshSupportModule::getSshSession(rUsername_, rHostname_, pSSHSession) || !pSSHSession)
    {
        success = false;
    }

    if (success && (!LibSshSupportModule::getSftpSessions(pSSHSession, pSftpSession) || !pSftpSession))
    {
        success = false;
    }

    std::string localFilePath = "";
    if (success && QDownloadedFileManager::getInstance().getFilePath(rFileName_, localFilePath))
    {
        localFile = std::make_unique<QFile>(localFilePath.c_str());
    }
    else
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"), "Couldn't find local file, flow error");
        success = false;
    }

    if (success && !localFile->open(QIODevice::ReadOnly))
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"), "Couldn't open local file for reading");
        success = false;
    }

    std::string remoteFilePath = rRemoteFolderPath_ + "/" + rFileName_;
    if (success && !(pfile = sftp_open(pSftpSession, remoteFilePath.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU)))
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"),
                           "Error opening remote file for writing: " << rUsername_ << "@" << rHostname_ << ":" << remoteFilePath
                                                                     << " (" << ssh_get_error(pSSHSession) << ")");
        success = false;
    }

    if (success)
    {
        char buffer[FILE_TRANSFER_BUFFER_SIZE] = {0};

        uint64_t totalBytesWritten = 0u;
        float progress = 0.0f;
        int64_t bytesRead = 0;
        ssize_t bytesWritten = 0u;

        while (success && !_cancelCurrentTasksFlag.load() && !localFile->atEnd())
        {
            if ((bytesRead = localFile->read(buffer, sizeof(buffer))) < 0)
            {
                success = false;
            }

            if (success && (bytesWritten = sftp_write(pfile, buffer, bytesRead)) < 0)
            {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"), "Error writing to remote file: " << ssh_get_error(pSSHSession));
                success = false;
            }

            if (success)
            {
                totalBytesWritten += bytesWritten;
                progress = 100.0f * static_cast<float>(totalBytesWritten) / static_cast<float>(localFile->size());

                auto tmp_clock = rclcpp::Clock();
                RCLCPP_INFO_THROTTLE(rclcpp::get_logger("GUI"),
                                     tmp_clock,
                                     1'000,
                                     "[ %f %%] Upload of %s",
                                     progress,
                                     remoteFilePath.c_str());
                emit this->newProgressBarUpdate(std::string(" Uploading ") + remoteFilePath + "...", progress);
            }
        }

        if (success && static_cast<int64_t>(totalBytesWritten) != localFile->size())
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"), "Error while uploading file, expect corrupt file");
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

    lockLibSsh.unlock();

    if (!_cancelCurrentTasksFlag.load() && !success)
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"),
                           rFileName_ << " upload to " << rUsername_ << "@" << rHostname_ << ":" << rRemoteFolderPath_
                                      << " was unsuccessful.");
    }
    else if (_cancelCurrentTasksFlag.load())
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"),
                           rFileName_ << " upload to " << rUsername_ << "@" << rHostname_ << ":" << rRemoteFolderPath_
                                      << " was canceled by the user.");
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
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error while opening file, no action done");
    }
}
