#ifndef __SSH_HANDLER_HPP__
#define __SSH_HANDLER_HPP__

#include <rclcpp/rclcpp.hpp>

#include <QDateTime>
#include <QMessageBox>

#include <libssh/libssh.h>
#include <libssh/sftp.h>

#include "QFileItem.hpp"
#include "rovus_lib/macros.h"

namespace SshHandler
{
    constexpr uint8_t MAX_LOGIN_ATTEMPT = 3u;

    bool handleAuth(IN const std::string& rUsername_, IN const std::string& rHostname_, OUT ssh_session& pSession_);
    bool askSshSetup(void);
    void sshSetupDialog(const std::string& rUsername_, const std::string& rHostname_);
    std::string getFileExtension(const std::string& filename_);
    std::string unixTimeToString(uint32_t unixTime_);
    void sortAttributeVector(INOUT std::vector<sftp_attributes>& vector);

    bool retrieveFolderStructure(IN const std::string& rUsername_,
                                 IN const std::string& rHostname_,
                                 IN const std::string& rPath,
                                 OUT std::vector<QFileItem>& rVecItems_)
    {
        ssh_session pSession = nullptr;

        if (!handleAuth(rUsername_, rHostname_, pSession) || !pSession)
        {
            ssh_disconnect(pSession);
            ssh_free(pSession);
            return false;
        }

        // Initialize SFTP session
        sftp_session sftp = sftp_new(pSession);
        if (sftp == nullptr)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("GUI"), "Error creating SFTP session: " << ssh_get_error(pSession));
            ssh_disconnect(pSession);
            ssh_free(pSession);
            return false;
        }

        int sshStatusCode = sftp_init(sftp);
        if (sshStatusCode != SSH_OK)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("GUI"), "Error initializing SFTP session: " << ssh_get_error(sftp));
            sftp_free(sftp);
            ssh_disconnect(pSession);
            ssh_free(pSession);
            return false;
        }

        // Retrieve the folder structure
        sftp_dir dir = sftp_opendir(sftp, std::string(rPath).c_str());
        if (dir == nullptr)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("GUI"), "Error opening directory: " << ssh_get_error(sftp));
            sftp_free(sftp);
            ssh_disconnect(pSession);
            ssh_free(pSession);
            return false;
        }

        sftp_attributes attrs;
        std::vector<sftp_attributes> filesAttribute;
        std::vector<sftp_attributes> folderAttribute;
        std::vector<sftp_attributes> otherAttribute;
        while ((attrs = sftp_readdir(sftp, dir)) != nullptr)
        {
            if (attrs->permissions & SSH_S_IFDIR)  // Files
            {
                if (std::string(attrs->name) != "." && std::string(attrs->name) != "..")
                {
                    filesAttribute.push_back(attrs);
                }
            }
            else if ((attrs->permissions & SSH_S_IFDIR))  // Dirs
            {
                folderAttribute.push_back(attrs);
            }
            else
            {
                otherAttribute.push_back(attrs);
            }
        }

        sortAttributeVector(filesAttribute);
        sortAttributeVector(folderAttribute);
        sortAttributeVector(otherAttribute);

        for (auto& it : filesAttribute)
        {
            rVecItems_.push_back(QFileItem(it->name, getFileExtension(it->name), unixTimeToString(it->mtime)));
        }

        for (auto& it : folderAttribute)
        {
            rVecItems_.push_back(QFileItem(it->name, "", unixTimeToString(it->mtime)));
        }

        for (auto& it : otherAttribute)
        {
            rVecItems_.push_back(QFileItem(it->name, "*", unixTimeToString(it->mtime)));
        }

        sftp_closedir(dir);
        sftp_free(sftp);
        ssh_disconnect(pSession);
        ssh_free(pSession);

        return true;
    }

    bool handleAuth(IN const std::string& rUsername_, IN const std::string& rHostname_, OUT ssh_session& pSession_)
    {
        int sshStatusCode = SSH_ERROR;
        for (uint8_t i = 0; sshStatusCode != SSH_AUTH_SUCCESS && i < MAX_LOGIN_ATTEMPT; i++)
        {
            pSession_ = ssh_new();
            if (pSession_ == nullptr)
            {
                RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error creating a ssh session");
                return false;
            }

            ssh_options_set(pSession_, SSH_OPTIONS_HOST, rHostname_.c_str());
            ssh_options_set(pSession_, SSH_OPTIONS_USER, rUsername_.c_str());

            sshStatusCode = ssh_connect(pSession_);
            if (sshStatusCode != SSH_OK)
            {
                RCLCPP_INFO(rclcpp::get_logger("GUI"), "Error connecting to host: %s", ssh_get_error(pSession_));
                ssh_free(pSession_);
                return false;
            }

            sshStatusCode = SSH_AUTH_ERROR;
            sshStatusCode = ssh_userauth_publickey_auto(pSession_, nullptr, nullptr);
            if (sshStatusCode == SSH_AUTH_SUCCESS)
            {
                break;
            }

            RCLCPP_INFO(rclcpp::get_logger("GUI"), "SSH key authentication failed with: %s", ssh_get_error(pSession_));

            if (!SshHandler::askSshSetup())
            {
                ssh_free(pSession_);
                return false;
            }

            SshHandler::sshSetupDialog(rUsername_, rHostname_);
            RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Setuping SSH Key...");
            continue;
        }

        if (sshStatusCode != SSH_AUTH_SUCCESS)
        {
            ssh_disconnect(pSession_);
            ssh_free(pSession_);
            return false;
        }

        return true;
    }

    void sshSetupDialog(const std::string& rUsername_, const std::string& rHostname_)
    {
        std::string message(std::string("For security reasons, we won't store password into memory.")
                            + " To setup a safe no password login (ssh key), "
                            + " please paste this command in a terminal and press \"ok\"\n\n" + "ssh-copy-id " + rUsername_ + "@"
                            + rHostname_);

        QMessageBox::question(nullptr, "Ssh Setup", message.c_str(), QMessageBox::Ok | QMessageBox::Cancel);
    }

    bool askSshSetup(void)
    {
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(nullptr,
                                      "SSH Key setup",
                                      "Logging with SSH key failed, do you want to setup one?",
                                      QMessageBox::Yes | QMessageBox::No);

        return reply == QMessageBox::Yes;
    }

    std::string getFileExtension(const std::string& filename_)
    {
        size_t lastDotPos = filename_.find_last_of('.');

        // Avoid first dot for hidden files
        if (lastDotPos != std::string::npos && lastDotPos != 0)
        {
            return filename_.substr(lastDotPos + 1);
        }

        return "";
    }

    std::string unixTimeToString(uint32_t unixTime_)
    {
        return QDateTime::fromSecsSinceEpoch(unixTime_).toString("yyyy/MM/dd HH:mm").toStdString();
    }

    void sortAttributeVector(INOUT std::vector<sftp_attributes>& vector)
    {
        std::sort(vector.begin(),
                  vector.end(),
                  [](const sftp_attributes& a, const sftp_attributes& b) { return std::string(a->name) < std::string(b->name); });
    }
};  // namespace SshHandler

#endif  // __SSH_HANDLER_HPP__
