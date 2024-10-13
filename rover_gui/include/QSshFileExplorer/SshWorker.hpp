#ifndef __SSH_HANDLER_HPP__
#define __SSH_HANDLER_HPP__

#include <rclcpp/rclcpp.hpp>

#include <QApplication>
#include <QDateTime>
#include <QMessageBox>
#include <QThread>

#include <libssh/libssh.h>
#include <libssh/sftp.h>

#include "QSshFileExplorer/QFileItem.hpp"
#include "rovus_lib/macros.h"

class SshWorker : public QThread
{
    Q_OBJECT

  private:
    static constexpr uint8_t MAX_LOGIN_ATTEMPT = 3u;

  public:
    SshWorker(QObject* parent_ = nullptr): QThread(parent_)
    {
        connect(this, &SshWorker::updateStructure, this, &SshWorker::getStructure);
    }

    ~SshWorker()
    {
        this->quit();
        this->wait();
    }

  signals:
    IN void updateStructure(IN const std::string& rUsername_, IN const std::string& rHostname_, IN const std::string& rPath);

    /**
     * @brief Connect to this signal to update the QItemModel when new data is available
     * @param files_ std::vector of QFileItem to add to your model
     * @return OUT
     */
    OUT void newDataReady(const std::vector<QFileItem>& files_);

  private:
    std::vector<QFileItem> _files = {};

    void run(void) override
    {
        this->exec();
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

            if (!askSshSetup())
            {
                ssh_free(pSession_);
                return false;
            }

            sshSetupDialog(rUsername_, rHostname_);
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
    bool askSshSetup(void)
    {
        QMessageBox::StandardButton reply;
        QMetaObject::invokeMethod(QApplication::instance(),
                                  [&]()
                                  {
                                      reply = QMessageBox::question(nullptr,
                                                                    "SSH Key setup",
                                                                    "Logging with SSH key failed, do you want to setup one?",
                                                                    QMessageBox::Yes | QMessageBox::No);
                                  });

        return reply == QMessageBox::Yes;
    }
    void sshSetupDialog(const std::string& rUsername_, const std::string& rHostname_)
    {
        std::string message(std::string("For security reasons, we won't store password into memory.") +
                            " To setup a safe no password login (ssh key), " +
                            " please paste this command in a terminal and press \"ok\"\n\n" + "ssh-copy-id " + rUsername_ + "@" +
                            rHostname_);

        QMetaObject::invokeMethod(
            QApplication::instance(),
            [&]() { QMessageBox::question(nullptr, "Ssh Setup", message.c_str(), QMessageBox::Ok | QMessageBox::Cancel); });
    }
    void sortAttributeVector(INOUT std::vector<sftp_attributes>& vector)
    {
        std::sort(vector.begin(),
                  vector.end(),
                  [](const sftp_attributes& a, const sftp_attributes& b) { return std::string(a->name) < std::string(b->name); });
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

  private slots:
    void getStructure(IN const std::string& rUsername_, IN const std::string& rHostname_, IN const std::string& rPath)
    {
        // RCLCPP_INFO(rclcpp::get_logger("GUI"), "Updating from %lu", std::hash<std::thread::id>{}(std::this_thread::get_id()));
        ssh_session pSession = nullptr;

        if (!handleAuth(rUsername_, rHostname_, pSession) || !pSession)
        {
            ssh_disconnect(pSession);
            ssh_free(pSession);
            return;
        }

        // Initialize SFTP session
        sftp_session sftp = sftp_new(pSession);
        if (sftp == nullptr)
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"), "Error creating SFTP session: " << ssh_get_error(pSession));
            ssh_disconnect(pSession);
            ssh_free(pSession);
            return;
        }

        int sshStatusCode = sftp_init(sftp);
        if (sshStatusCode != SSH_OK)
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"), "Error initializing SFTP session: " << ssh_get_error(sftp));
            sftp_free(sftp);
            ssh_disconnect(pSession);
            ssh_free(pSession);
            return;
        }

        // Retrieve the folder structure
        sftp_dir dir = sftp_opendir(sftp, std::string(rPath).c_str());
        if (dir == nullptr)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("GUI"), "Error opening directory: " << ssh_get_error(sftp));
            sftp_free(sftp);
            ssh_disconnect(pSession);
            ssh_free(pSession);
            return;
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

            sftp_attributes_free(attrs);
        }

        sortAttributeVector(filesAttribute);
        sortAttributeVector(folderAttribute);
        sortAttributeVector(otherAttribute);

        _files.clear();
        for (auto& it : filesAttribute)
        {
            _files.push_back(QFileItem(it->name, getFileExtension(it->name), unixTimeToString(it->mtime)));
        }

        for (auto& it : folderAttribute)
        {
            _files.push_back(QFileItem(it->name, "", unixTimeToString(it->mtime)));
        }

        for (auto& it : otherAttribute)
        {
            _files.push_back(QFileItem(it->name, "*", unixTimeToString(it->mtime)));
        }

        sftp_closedir(dir);
        sftp_free(sftp);
        ssh_disconnect(pSession);
        ssh_free(pSession);

        emit this->newDataReady(_files);
    }
};

#endif  // __SSH_HANDLER_HPP__
