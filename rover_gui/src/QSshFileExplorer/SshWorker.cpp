#include "SshWorker.hpp"

SshWorker::SshWorker(bool start_, QObject* parent_): Worker(start_, parent_) {}

SshWorker::~SshWorker()
{
    this->finish();
}

void SshWorker::refreshStructure(std::string username_, std::string hostname_, std::string path_)
{
    this->addTask([username = std::move(username_), hostname = std::move(hostname_), path = std::move(path_), this](void)
                  { this->refreshStructureInternal(username, hostname, path); });
}

std::vector<QFileItem> SshWorker::getStructure(void)
{
    std::lock_guard<std::mutex> lock(_filesMutex);
    return _files;
}

bool SshWorker::handleAuth(IN const std::string& rUsername_, IN const std::string& rHostname_, OUT ssh_session& pSession_)
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
            RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error connecting to host: %s", ssh_get_error(pSession_));
            ssh_free(pSession_);
            return false;
        }

        sshStatusCode = SSH_AUTH_ERROR;
        sshStatusCode = ssh_userauth_publickey_auto(pSession_, nullptr, nullptr);
        if (sshStatusCode == SSH_AUTH_SUCCESS)
        {
            break;
        }

        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "SSH key authentication failed with: %s", ssh_get_error(pSession_));

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

bool SshWorker::askSshSetup(void)
{
    QEventLoop waitForAnswer;
    QMessageBox::StandardButton reply;
    QMetaObject::invokeMethod(QApplication::instance(),
                              [&]()
                              {
                                  reply = QMessageBox::question(nullptr,
                                                                "SSH Key setup",
                                                                "Logging with SSH key failed, do you want to setup one?",
                                                                QMessageBox::Yes | QMessageBox::No);
                                  waitForAnswer.quit();
                              });

    waitForAnswer.exec();
    return reply == QMessageBox::Yes;
}

void SshWorker::sshSetupDialog(const std::string& rUsername_, const std::string& rHostname_)
{
    QEventLoop waitForAnswer;
    std::string message(std::string("For security reasons, we won't store password into memory.") +
                        " To setup a safe no password login (ssh key), " +
                        " please paste this command in a terminal and press \"ok\"\n\n" + "ssh-copy-id " + rUsername_ + "@" +
                        rHostname_);

    QMetaObject::invokeMethod(
        QApplication::instance(),
        [&]()
        {
            QMessageBox::question(nullptr, "Ssh Setup", message.c_str(), QMessageBox::Ok | QMessageBox::Cancel);
            waitForAnswer.quit();
        });

    waitForAnswer.exec();
}

void SshWorker::sortAttributeVector(INOUT std::vector<sftp_attributes>& vector)
{
    std::sort(vector.begin(),
              vector.end(),
              [](const sftp_attributes& a, const sftp_attributes& b) { return std::string(a->name) < std::string(b->name); });
}

std::string SshWorker::getFileExtension(const std::string& filename_)
{
    size_t lastDotPos = filename_.find_last_of('.');

    // Avoid first dot for hidden files
    if (lastDotPos != std::string::npos && lastDotPos != 0)
    {
        return filename_.substr(lastDotPos + 1);
    }

    return "";
}

std::string SshWorker::unixTimeToString(uint32_t unixTime_)
{
    return QDateTime::fromSecsSinceEpoch(unixTime_).toString("yyyy/MM/dd HH:mm").toStdString();
}

void SshWorker::refreshStructureInternal(std::string username_, std::string hostname_, std::string path_)
{
    ssh_session pSession = nullptr;

    if (!handleAuth(username_, hostname_, pSession) || !pSession)
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
        if (attrs->permissions & SSH_S_IFDIR)  // Files
        {
            if (std::string(attrs->name) != ".")
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

    {
        std::unique_lock<std::mutex> lock(_filesMutex);
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
    }

    sftp_closedir(dir);
    sftp_free(sftp);
    ssh_disconnect(pSession);
    ssh_free(pSession);

    emit this->newStructureReady();
}
