#include "QSshWorker.hpp"

#include <fcntl.h>
#include <rclcpp/rclcpp.hpp>

#include <QApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QMessageBox>
#include <QUuid>

#include "../Global/Helpers/QHelpers.hpp"
#include "../Global/QTmpFolderManager.hpp"
#include "QDownloadedFileManager.hpp"
#include "rovus_lib/macros.h"

#warning TODO: Link correct path with OPEN

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

void QSshWorker::downloadFile(IN const std::string& rUsername_,
                              IN const std::string& rHostname_,
                              IN const std::string& rfilePath_)
{
    this->addTask([username = rUsername_, hostname = rHostname_, path = rfilePath_, this](void)
                  { this->downloadFileInternal(username, hostname, path); });
}

std::vector<QFileItem> QSshWorker::getStructure(void)
{
    std::lock_guard<std::mutex> lock(_filesMutex);
    return _files;
}

void QSshWorker::refreshStructureInternal(std::string username_, std::string hostname_, std::string path_)
{
#warning TODO Refactor
    ssh_session pSession = nullptr;

    if (!getSshSession(username_, hostname_, pSession) || !pSession)
    {
        ssh_disconnect(pSession);
        return;
    }

    // Initialize SFTP session
    sftp_session sftp = sftp_new(pSession);
    if (sftp == nullptr)
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"), "Error creating SFTP session: " << ssh_get_error(pSession));
        ssh_disconnect(pSession);
        return;
    }

    int sshStatusCode = sftp_init(sftp);
    if (sshStatusCode != SSH_OK)
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"), "Error initializing SFTP session: " << ssh_get_error(sftp));
        sftp_free(sftp);
        ssh_disconnect(pSession);
        return;
    }

    // Retrieve the folder structure
    sftp_dir dir = sftp_opendir(sftp, path_.c_str());
    if (dir == nullptr)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("GUI"), "Error opening directory: " << ssh_get_error(sftp));
        sftp_free(sftp);
        ssh_disconnect(pSession);
        return;
    }

    sftp_attributes attrs;
    std::vector<sftp_attributes> filesAttribute;
    std::vector<sftp_attributes> folderAttribute;
    std::vector<sftp_attributes> otherAttribute;
    while ((attrs = sftp_readdir(sftp, dir)) && attrs->name && attrs->permissions)
    {
        if (attrs->permissions & SSH_S_IFDIR)
        {
            if (std::string(attrs->name) != ".")
            {
                folderAttribute.push_back(attrs);
            }
        }
        else if (!(attrs->permissions & SSH_S_IFDIR))
        {
            filesAttribute.push_back(attrs);
        }
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
        }
        for (auto& it : filesAttribute)
        {
            _files.push_back(QFileItem(it->name, getFileExtension(it->name), unixTimeToString(it->mtime)));
        }
        for (auto& it : otherAttribute)
        {
            _files.push_back(QFileItem(it->name, "*", unixTimeToString(it->mtime)));
        }
    }

    sftp_closedir(dir);
    sftp_free(sftp);
    ssh_disconnect(pSession);
    ssh_free(pSession);

    emit this->newStructureReady();
}

void QSshWorker::downloadFileInternal(IN const std::string& rUsername_,
                                      IN const std::string& rHostname_,
                                      IN const std::string& rRemoteFilePath_)
{
    bool success = true;

    ssh_session pSSHSession = nullptr;
    sftp_session pSftpSession = nullptr;
    sftp_file pfile = nullptr;

    if (!this->getSshSession(rUsername_, rHostname_, pSSHSession) || !pSSHSession)
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error getting SSH session, no file will be transferred");
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
        case QDownloadedFileManager::eDownloadState::ALREADY_DOWNLOADED_ERROR:
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
        uint8_t buffer[4096] = {0};
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
    }
    if (pSftpSession)
    {
        sftp_free(pSftpSession);
    }
    if (pSSHSession)
    {
        ssh_free(pSSHSession);
    }

    if (!success)
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"),
                           "File transfer of " << rUsername_ << "@" << rHostname_ << ":" << rRemoteFilePath_
                                               << " was unsuccessful.");
    }
}

// ===============================================================================================================================
// Helpers
#warning TODO: Create Class/Namespace with helpers
// ===============================================================================================================================
bool QSshWorker::getSshSession(IN const std::string& rUsername_, IN const std::string& rHostname_, OUT ssh_session& pSshSession_)
{
    if (rUsername_ == "" || rHostname_ == "")
    {
        return false;
    }

    bool success = true;
    int sshStatusCode = SSH_ERROR;
    for (uint8_t i = 0; success && sshStatusCode != SSH_AUTH_SUCCESS && i < MAX_LOGIN_ATTEMPT; i++)
    {
        pSshSession_ = ssh_new();
        if (success && !pSshSession_)
        {
            sshStatusCode = SSH_ERROR;
            success = false;
        }
        else if (success)
        {
            ssh_options_set(pSshSession_, SSH_OPTIONS_TIMEOUT_USEC, &LOGIN_TIMEOUT);
            ssh_options_set(pSshSession_, SSH_OPTIONS_HOST, rHostname_.c_str());
            ssh_options_set(pSshSession_, SSH_OPTIONS_USER, rUsername_.c_str());
            sshStatusCode = ssh_connect(pSshSession_);
        }

        if (success && sshStatusCode != SSH_OK && !this->handleSshSetup(rUsername_, rHostname_))
        {
            RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error connecting to host: %s", ssh_get_error(pSshSession_));
            success = false;
        }

        sshStatusCode = ssh_userauth_publickey_auto(pSshSession_, nullptr, nullptr);
        if (success && sshStatusCode != SSH_AUTH_SUCCESS && !this->sshSetupDialog(rUsername_, rHostname_))
        {
            success = false;
        }
        else if (success)
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
            RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"), "Error creating SFTP session: " << ssh_get_error(pSshSession_));
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
