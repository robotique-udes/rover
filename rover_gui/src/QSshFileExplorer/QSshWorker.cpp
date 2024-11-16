#include "QSshWorker.hpp"

#include <fcntl.h>
#include <rclcpp/rclcpp.hpp>

#include <QApplication>
#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QFile>
#include <QMessageBox>
#include <QUrl>
#include <QUuid>

#include "../Global/Helpers/QHelpers.hpp"
#include "../Global/QTmpFolderManager.hpp"
#include "QDownloadedFileManager.hpp"
#include "rovus_lib/macros.h"

#warning TODO: Link correct path with OPEN

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

    this->addTask([path = getFileNameFromPath(rfilePath_), this](void) { this->openLocalFile(path); });
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
    if (!getSshSession(username_, hostname_, pSshSession) || !pSshSession)
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"), "Error creating SSH session, device ssh service might not be running");
        success = false;
    }

    if (success && (!pSshSession || !getSftpSessions(pSshSession, pSftpSession) || !pSftpSession))
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
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("GUI"), "Error opening directory: " << ssh_get_error(pSftpSession));
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

        sortAttributeVector(folderAttribute);
        sortAttributeVector(filesAttribute);
        sortAttributeVector(otherAttribute);

        {
            std::unique_lock<std::mutex> lock(_filesMutex);
            _files.clear();

            for (auto& it : folderAttribute)
            {
                _files.push_back(QFileItem(it->name, "", unixTimeToString(it->mtime)));
                sftp_attributes_free(it);
            }
            for (auto& it : filesAttribute)
            {
                _files.push_back(QFileItem(it->name, getFileExtension(it->name), unixTimeToString(it->mtime)));
                sftp_attributes_free(it);
            }
            for (auto& it : otherAttribute)
            {
                _files.push_back(QFileItem(it->name, "*", unixTimeToString(it->mtime)));
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
    if (!this->getSshSession(rUsername_, rHostname_, pSSHSession) || !pSSHSession)
    {
        success = false;
    }

    if (success && (!this->getSftpSessions(pSSHSession, pSftpSession) || !pSftpSession))
    {
        success = false;
    }

    uint64_t fileSize = 0u;
    success = this->getFileSize(pSftpSession, rRemoteFilePath_, fileSize);

    switch (QDownloadedFileManager::getInstance().alreadyDownloaded(this->getFileNameFromPath(rRemoteFilePath_), fileSize))
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

    std::string fileName = this->getFileNameFromPath(rRemoteFilePath_);
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
                                                << " Download of " << rRemoteFilePath_ << " in progress");
            }
        }

        if (nbytes >= 0)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("GUI"), "Finished download of " << rRemoteFilePath_ << " successfully");
            QDownloadedFileManager::getInstance().addFileToList(this->getFileNameFromPath(rRemoteFilePath_), fileSize);
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
        // TODO: Print fail
    }
}

// ===============================================================================================================================
// Helpers
#warning TODO: Create Class/Namespace with helpers
// ===============================================================================================================================
bool QSshWorker::getSshSession(IN const std::string& rUsername_, IN const std::string& rHostname_, OUT ssh_session& pSshSession_)
{
    if (rUsername_ == "" || rHostname_ == "" || pSshSession_)
    {
        return false;
    }

    bool success = true;
    int sshStatusCode = SSH_ERROR;
    for (uint8_t i = 0; success && sshStatusCode != SSH_AUTH_SUCCESS && i < MAX_LOGIN_ATTEMPT; i++)
    {
        pSshSession_ = ssh_new();
        if (pSshSession_)
        {
            ssh_options_set(pSshSession_, SSH_OPTIONS_TIMEOUT_USEC, &LOGIN_TIMEOUT);
            ssh_options_set(pSshSession_, SSH_OPTIONS_HOST, rHostname_.c_str());
            ssh_options_set(pSshSession_, SSH_OPTIONS_USER, rUsername_.c_str());
        }

        if (pSshSession_ && ssh_connect(pSshSession_) == SSH_OK)
        {
            sshStatusCode = ssh_userauth_publickey_auto(pSshSession_, nullptr, nullptr);
        }
        else
        {
            this->handleSshSetup(rUsername_, rHostname_);
            sshStatusCode = SSH_ERROR;
            success = false;
        }

        if (success && sshStatusCode != SSH_AUTH_SUCCESS && !this->handleSshSetup(rUsername_, rHostname_))
        {
            RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error connecting to host: %s", ssh_get_error(pSshSession_));
            success = false;
        }

        if (success && sshStatusCode != SSH_AUTH_SUCCESS)
        {
            sshStatusCode = ssh_userauth_publickey_auto(pSshSession_, nullptr, nullptr);
        }

        if (success && sshStatusCode != SSH_AUTH_SUCCESS && !this->sshSetupDialog(rUsername_, rHostname_))
        {
            // Connection setup refused by user
            success = false;
        }
        else if (success && sshStatusCode != SSH_AUTH_SUCCESS)
        {
            sshStatusCode = ssh_userauth_publickey_auto(pSshSession_, nullptr, nullptr);
        }

        if (!success && sshStatusCode != SSH_AUTH_SUCCESS && pSshSession_)
        {
            ssh_disconnect(pSshSession_);
            ssh_free(pSshSession_);
            pSshSession_ = nullptr;
        }
    }

    return success;
}

bool QSshWorker::getSftpSessions(INOUT ssh_session& pSshSession_, OUT sftp_session& pSftpSession_)
{
    bool success = true;

    if (success && !pSshSession_)
    {
        success = false;
    }

    if (success)
    {
        pSftpSession_ = sftp_new(pSshSession_);
        if (!pSftpSession_)
        {
            success = false;
        }
    }

    if (success && sftp_init(pSftpSession_) != SSH_OK)
    {
        sftp_free(pSftpSession_);
        pSftpSession_ = nullptr;
        success = false;
    }

    return success;
}

bool QSshWorker::handleSshSetup(const std::string& rUsername_, const std::string& rHostname_) const
{
    bool success = true;

    if (success && rHostname_ == "localhost")
    {
        QMessageBox::StandardButton userSelection = QMessageBox::StandardButton::No;
        userSelection = QPopUp::sendQuestionPopUp("SSH Connection error",
                                                  "Couldn't connect to server " + rUsername_ + "@" + rHostname_
                                                      + ". The openssh server might not be running. Do you want to enable it?",
                                                  (QMessageBox::StandardButton::Yes | QMessageBox::StandardButton::No));

        if (userSelection == QMessageBox::StandardButton::Yes)
        {
            std::string result;
            // Using timeout of 20 seconds because the user might be prompted to enter his password
            if (!QTerminalCommand::blockingTerminalCommand("systemctl", {"start", "ssh"}, result, std::chrono::seconds(20)))
            {
                success = false;
            }
        }
    }
    else if (success)
    {
        QPopUp::sendQuestionPopUp(
            "SSH Connection error",
            "Couldn't connect to server " + rUsername_ + "@" + rHostname_
                + ". The openssh server might not be running. Enter this command on the server terminal to start "
                  "it:"
                + "\n\nsudo systemctl start ssh"
                + "\n\nTo enable it at startup, enter this command:" + "\n\nsudo systemctl enable --now ssh");
    }

    return success;
}

bool QSshWorker::sshSetupDialog(const std::string& rUsername_, const std::string& rHostname_) const
{
    QMessageBox::StandardButton userInput = QMessageBox::StandardButton::No;
    userInput = QPopUp::sendQuestionPopUp("SSH Key setup",
                                          std::string("For security reasons, we won't store password into memory.")
                                              + " To setup a safe no password login (ssh key), "
                                              + " please paste this command in a terminal and press \"ok\"\n\n" + "ssh-copy-id "
                                              + rUsername_ + "@" + rHostname_,
                                          (QMessageBox::StandardButton::Ok | QMessageBox::StandardButton::Cancel));

    return userInput == QMessageBox::StandardButton::Ok;
}

void QSshWorker::sortAttributeVector(INOUT std::vector<sftp_attributes>& vector_) const
{
    std::sort(vector_.begin(),
              vector_.end(),
              [](const sftp_attributes& a, const sftp_attributes& b) { return std::string(a->name) < std::string(b->name); });
}

std::string QSshWorker::getFileExtension(const std::string& filename_) const
{
    size_t lastDotPos = filename_.find_last_of('.');

    // Avoid first dot for hidden files
    if (lastDotPos != std::string::npos && lastDotPos != 0)
    {
        return filename_.substr(lastDotPos + 1);
    }

    return "";
}

std::string QSshWorker::unixTimeToString(const uint32_t unixTime_) const
{
    return QDateTime::fromSecsSinceEpoch(unixTime_).toString("yyyy/MM/dd HH:mm").toStdString();
}

bool QSshWorker::getFileSize(INOUT sftp_session sftp_, const std::string& rFilePath, OUT uint64_t& fileSize_)
{
    sftp_attributes fileAttributes = nullptr;
    fileSize_ = 0;
    bool success = true;

    success = CHECK_POINTER_VALID(sftp_);

    if (success)
    {
        fileAttributes = sftp_stat(sftp_, rFilePath.c_str());
        success = CHECK_POINTER_VALID(fileAttributes);
    }

    if (fileAttributes)
    {
        fileSize_ = fileAttributes->size;
        sftp_attributes_free(fileAttributes);
        fileAttributes = nullptr;
    }

    return success;
}

std::string QSshWorker::getFileNameFromPath(const std::string& path_)
{
    return QFileInfo(QString::fromStdString(path_)).fileName().toStdString();
}
