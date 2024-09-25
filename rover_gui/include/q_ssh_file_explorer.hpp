#pragma once

#include <QHeaderView>
#include <QInputDialog>
#include <QMessageBox>
#include <libssh/libssh.h>
#include <libssh/sftp.h>
#include <rclcpp/rclcpp.hpp>

#include "rovus_lib/macros.h"

#include "q_file_item.hpp"

namespace SshClient
{
    void sshSetupDialog(const std::string &rUsername_, const std::string &rHostname_)
    {
        std::string message(std::string("For security reasons, to setup your ssh key,") +
                            "please paste this command in a terminal and press \"ok\"\n\n" +
                            "ssh-copy-id " +
                            rUsername_ +
                            "@" +
                            rHostname_);

        QMessageBox::question(nullptr,
                              "Ssh Setup",
                              message.c_str(),
                              QMessageBox::StandardButton::Ok);
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

    void retrieveFolderStructure(IN const std::string &rHostname_,
                                 IN const std::string &rUsername_,
                                 OUT std::vector<QFileItem> &rFileItems_)
    {
        ssh_session pSession = ssh_new();
        if (pSession == nullptr)
        {
            RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error creating a ssh session");
            return;
        }

        ssh_options_set(pSession, SSH_OPTIONS_HOST, rHostname_.c_str());
        ssh_options_set(pSession, SSH_OPTIONS_USER, rUsername_.c_str());

        int rc = ssh_connect(pSession);
        if (rc != SSH_OK)
        {
            RCLCPP_INFO(rclcpp::get_logger("GUI"), "Error connecting to host: %s", ssh_get_error(pSession));
            ssh_free(pSession);
            return;
        }

        // Attempt to authenticate with an SSH key
        rc = ssh_userauth_publickey_auto(pSession, nullptr, nullptr);
        if (rc != SSH_AUTH_SUCCESS)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "SSH key authentication failed with: %s", ssh_get_error(pSession));

            if (SshClient::askSshSetup())
            {
                RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Setuping SSH Key...");
                SshClient::sshSetupDialog(rUsername_, rHostname_);
            }

#error Jsuis rendu là dans le pipeline

            rc = ssh_userauth_password(pSession, nullptr, "");
            if (rc != SSH_AUTH_SUCCESS)
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("GUI"), "Password authentication failed: " << ssh_get_error(pSession));
                ssh_disconnect(pSession);
                ssh_free(pSession);
                return;
            }
        }

        // Initialize SFTP session
        sftp_session sftp = sftp_new(pSession);
        if (sftp == nullptr)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("GUI"), "Error creating SFTP session: " << ssh_get_error(pSession));
            ssh_disconnect(pSession);
            ssh_free(pSession);
            return;
        }

        rc = sftp_init(sftp);
        if (rc != SSH_OK)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("GUI"), "Error initializing SFTP session: " << ssh_get_error(sftp));
            sftp_free(sftp);
            ssh_disconnect(pSession);
            ssh_free(pSession);
            return;
        }

        // Retrieve the folder structure
        sftp_dir dir = sftp_opendir(sftp, ".");
        if (dir == nullptr)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("GUI"), "Error opening directory: " << ssh_get_error(sftp));
            sftp_free(sftp);
            ssh_disconnect(pSession);
            ssh_free(pSession);
            return;
        }

        sftp_attributes attrs;
        while ((attrs = sftp_readdir(sftp, dir)) != nullptr)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("GUI"), "File: " << attrs->name);
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

        std::vector<QFileItem> files;
        SshClient::retrieveFolderStructure("localhost", "phil", files);
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
