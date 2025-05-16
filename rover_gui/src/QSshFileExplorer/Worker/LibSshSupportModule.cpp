#include "LibSshSupportModule.hpp"

#include <QDateTime>
#include <QMessageBox>

#include "Global/Helpers/QHelpers.hpp"
#include "Global/Helpers/QToastNotification/QToastNotification.hpp"

namespace LibSshSupportModule
{

    bool getSshSession(IN const std::string& rUsername_, IN const std::string& rHostname_, OUT ssh_session& pSshSession_)
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
                SshNoConnectionDialog(rUsername_, rHostname_);
                sshStatusCode = SSH_ERROR;
                success = false;
            }

            if (success && sshStatusCode != SSH_AUTH_SUCCESS && !SshNoConnectionDialog(rUsername_, rHostname_))
            {
                RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error connecting to host: %s", ssh_get_error(pSshSession_));
                success = false;
            }

            if (success && sshStatusCode != SSH_AUTH_SUCCESS)
            {
                sshStatusCode = ssh_userauth_publickey_auto(pSshSession_, nullptr, nullptr);
            }

            if (success && sshStatusCode != SSH_AUTH_SUCCESS && !sshSetupDialog(rUsername_, rHostname_))
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

    bool getSftpSessions(INOUT ssh_session& pSshSession_, OUT sftp_session& pSftpSession_)
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

    bool SshNoConnectionDialog(const std::string& rUsername_, const std::string& rHostname_)
    {
        bool success = true;

        if (success && rHostname_ == "localhost")
        {
            QHelper::QToastNotification::getInstance().notifyFromAnyThread(
                "Background task started",
                "SSH Service is being started in the background, you might get prompted for your password",
                QHelper::QToastNotification::eNotifType::INFO);

            std::string result;
            // Using timeout of 10 seconds because the user might be prompted to enter his password
            if (!QHelper::QTerminalCommand::blockingTerminalCommand("systemctl",
                                                                    {"start", "ssh"},
                                                                    result,
                                                                    std::chrono::seconds(10)))
            {
                success = false;
            }
        }
        else if (success)
        {
            QHelper::QToastNotification::getInstance().notifyFromAnyThread(
                "SSH Connection error",
                "Couldn't connect to server " + rUsername_ + "@" + rHostname_
                    + ". The openssh server might not be running. Enter this command on the server terminal to start it: sudo "
                      "systemctl enable --now ssh",
                QHelper::QToastNotification::eNotifType::WARNING,
                5'000);
        }

        return success;
    }

    bool sshSetupDialog(const std::string& rUsername_, const std::string& rHostname_)
    {
        QMessageBox::StandardButton userInput = QMessageBox::StandardButton::No;
        userInput = QHelper::QPopUp::sendQuestionPopUp("SSH Key setup",
                                                       std::string("For security reasons, we won't store password into memory.")
                                                           + " To setup a safe no password login (ssh key), "
                                                           + " please paste this command in a terminal and press \"ok\"\n\n"
                                                           + "ssh-copy-id " + rUsername_ + "@" + rHostname_,
                                                       (QMessageBox::StandardButton::Ok | QMessageBox::StandardButton::Cancel));

        return userInput == QMessageBox::StandardButton::Ok;
    }

    void sortSftpAttributeVector(INOUT std::vector<sftp_attributes>& vector_)
    {
        std::sort(vector_.begin(),
                  vector_.end(),
                  [](const sftp_attributes& a, const sftp_attributes& b)
                  {
                      return std::string(a->name) < std::string(b->name);
                  });
    }

    std::string unixTimeToString(const uint32_t unixTime_)
    {
        return QDateTime::fromSecsSinceEpoch(unixTime_).toString("yyyy/MM/dd HH:mm").toStdString();
    }

    bool getSftpFileSize(INOUT sftp_session sftp_, const std::string& rFilePath, OUT uint64_t& fileSize_)
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
}  // namespace LibSshSupportModule
