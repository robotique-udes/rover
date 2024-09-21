#pragma once

#include <QHeaderView>
#include <QInputDialog>
#include <libssh/libssh.h>
#include <libssh/sftp.h>
#include <rclcpp/rclcpp.hpp>
// #include <mutex>

#include "q_file_item.hpp"

class SshClient
{
    // static std::mutex mutexLibSsh = ;
public:
    SshClient() {}
    ~SshClient() {}

    void retrieveFolderStructure(const QString &host, const QString &user)
    {
        ssh_session pSession = ssh_new();
        if (pSession == nullptr)
            return;

        ssh_options_set(pSession, SSH_OPTIONS_HOST, host.toUtf8().constData());
        ssh_options_set(pSession, SSH_OPTIONS_USER, user.toUtf8().constData());

        int rc = ssh_connect(pSession);
        if (rc != SSH_OK)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("GUI"), "Error connecting to host:" << ssh_get_error(pSession));
            ssh_free(pSession);
            return;
        }

        QString password = QInputDialog::getText(nullptr, "Password", "Enter your password:", QLineEdit::Password);
        rc = ssh_userauth_password(pSession, nullptr, password.toUtf8().constData());
        if (rc != SSH_AUTH_SUCCESS)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("GUI"), "Authentication failed:" << ssh_get_error(pSession));
            ssh_disconnect(pSession);
            ssh_free(pSession);
            return;
        }

        // Initialize SFTP session
        sftp_session sftp = sftp_new(pSession);
        if (sftp == nullptr)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("GUI"), "Error creating SFTP session:" << ssh_get_error(pSession));
            ssh_disconnect(pSession);
            ssh_free(pSession);
            return;
        }

        rc = sftp_init(sftp);
        if (rc != SSH_OK)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("GUI"), "Error initializing SFTP session:" << ssh_get_error(sftp));
            sftp_free(sftp);
            ssh_disconnect(pSession);
            ssh_free(pSession);
            return;
        }

        // Retrieve the folder structure
        sftp_dir dir = sftp_opendir(sftp, ".");
        if (dir == nullptr)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("GUI"), "Error opening directory:" << ssh_get_error(sftp));
            sftp_free(sftp);
            ssh_disconnect(pSession);
            ssh_free(pSession);
            return;
        }

        sftp_attributes attrs;
        while ((attrs = sftp_readdir(sftp, dir)) != nullptr)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("GUI"), "File:" << attrs->name);
            sftp_attributes_free(attrs);
        }

        sftp_closedir(dir);
        sftp_free(sftp);
        ssh_disconnect(pSession);
        ssh_free(pSession);
    }
};

class QSshFileExplorer
{
private:
    enum class eColumnIndex : uint8_t
    {
        NAME,
        TYPE,
        LAST_MODIFIED
    };

    static const QString STYLE_TREE_VIEW;

public:
    QSshFileExplorer(QTreeView &rTreeView_) : _rTreeView(rTreeView_)
    {
        _rTreeView.setModel(&_itemModel);
        QFont fountSize;
        fountSize.setPointSize(12);
        _rTreeView.setFont(fountSize);
        QString currentStyle = _rTreeView.styleSheet();
        currentStyle.append(STYLE_TREE_VIEW);
        rTreeView_.setStyleSheet(currentStyle);

        _itemModel.setHorizontalHeaderLabels({"Name", "Type", "Last modified"});

        _rTreeView.header()->setStretchLastSection(false);
        _rTreeView.header()->setSectionResizeMode((uint8_t)eColumnIndex::NAME, QHeaderView::Stretch);

        _rTreeView.setColumnWidth((uint8_t)eColumnIndex::TYPE, 50);
        _rTreeView.header()->setSectionResizeMode((uint8_t)eColumnIndex::TYPE, QHeaderView::Fixed);

        _rTreeView.setColumnWidth((uint8_t)eColumnIndex::LAST_MODIFIED, 120);
        _rTreeView.header()->setSectionResizeMode((uint8_t)eColumnIndex::LAST_MODIFIED, QHeaderView::Fixed);

        this->refreshItems();

        SshClient sshHandler;
        sshHandler.retrieveFolderStructure("localhost", "phil");
    }

    bool refreshItems(void)
    {
        _itemModel.removeRows(1, _itemModel.rowCount());
        _itemModel.removeRows(1, _itemModel.rowCount());

        std::list<QFileItem> items;
        this->insertGoBackItem(items);

#warning TODO: Implement ssh ls
        // for (const auto &[extension, icon] : IconManager::getIconMap())
        // {
        //     // Generate a name for each item (you might want to customize this)
        //     std::string name = extension.empty() ? "Folder" : "File " + extension;

        //     // Create a QFileItem instance and add it to the list
        //     items.push_back(QFileItem(name, extension, "2024-13-17"));
        // }

        for (auto &it : items)
        {
            it.addItemToModel(_itemModel);
        }

        return true;
    }

    void insertGoBackItem(std::list<QFileItem> &items)
    {
        if (items.size() != 0 && items.front().getName() == QFileItemGoBack().getName())
        {
            return;
        }
        items.emplace_front(QFileItemGoBack());
    }

private:
    QTreeView &_rTreeView;
    QStandardItemModel _itemModel;
};

const QString QSshFileExplorer::STYLE_TREE_VIEW =
    R"(
        QTreeView::item {
            padding: 5px;
    })";
