#pragma once

#include <QDateTime>
#include <QHeaderView>
#include <QInputDialog>
#include <QMessageBox>
#include <QTreeView>

#include "QFileItem.hpp"
#include "SshWorker.hpp"

#include "UI_FileExplorer.h"

class QSshFileExplorerWidget : public QWidget
{
  private:
    enum class eColumnIndex : uint8_t
    {
        NAME = 0,
        TYPE,
        LAST_MODIFIED,
        eLAST
    };

    static const QString STYLE_TREE_VIEW;

  public:
    QSshFileExplorerWidget(QWidget* parent = nullptr): QWidget(parent)
    {
        ui.setupUi(this);

        ui.tv_fileExplorer->setModel(&_itemModel);
        QFont fountSize;
        fountSize.setPointSize(11);
        ui.tv_fileExplorer->setFont(fountSize);
        QString currentStyle = ui.tv_fileExplorer->styleSheet();
        currentStyle.append(STYLE_TREE_VIEW);
        ui.tv_fileExplorer->setStyleSheet(currentStyle);

        _itemModel.setHorizontalHeaderLabels({"Name", "Type", "Last modified"});

        ui.tv_fileExplorer->header()->setStretchLastSection(false);
        ui.tv_fileExplorer->header()->setSectionResizeMode((uint8_t)eColumnIndex::NAME, QHeaderView::Stretch);
        ui.tv_fileExplorer->setColumnWidth((uint8_t)eColumnIndex::TYPE, 50);
        ui.tv_fileExplorer->header()->setSectionResizeMode((uint8_t)eColumnIndex::TYPE, QHeaderView::Fixed);
        ui.tv_fileExplorer->setColumnWidth((uint8_t)eColumnIndex::LAST_MODIFIED, 150);
        ui.tv_fileExplorer->header()->setSectionResizeMode((uint8_t)eColumnIndex::LAST_MODIFIED, QHeaderView::Fixed);

        this->refreshItems();

        connect(ui.cb_showHiddenFile, &QCheckBox::stateChanged, this, &QSshFileExplorerWidget::refreshItems);
        connect(ui.pb_refresh, &QPushButton::clicked, this, &QSshFileExplorerWidget::refreshItems);
        _sshWorkerThread.start();
    }

    ~QSshFileExplorerWidget() {}

    bool refreshItems(void)
    {
        _itemModel.removeRows(0, _itemModel.rowCount());

        std::vector<QFileItem> files;

        // RCLCPP_INFO(rclcpp::get_logger("GUI"), "Calling from %lu", std::hash<std::thread::id>{}(std::this_thread::get_id()));
        RCLCPP_INFO(rclcpp::get_logger("GUI"), "Updating from %lu", std::hash<std::thread::id>{}(std::this_thread::get_id()));
        emit _sshWorkerThread.updateStructure("phil", "localhost", "/");

        RCLCPP_INFO(rclcpp::get_logger("GUI"), "Start");
        bool showHiddenFiles = ui.cb_showHiddenFile->isChecked();
        for (auto& it : files)
        {
            RCLCPP_INFO(rclcpp::get_logger("GUI"), it.getName().c_str());
            it.addItemToModel(_itemModel, showHiddenFiles);
        }
        RCLCPP_INFO(rclcpp::get_logger("GUI"), "Stop");

        return true;
    }

    void insertGoBackItem(std::list<QFileItem>& items)
    {
        if (items.size() != 0 && items.front().getName() == QFileItemGoBack().getName())
        {
            return;
        }
        items.emplace_front(QFileItemGoBack());
    }

    const Ui::FileExplorer& getUI(void)
    {
        return ui;
    }

  private:
    static const std::map<eColumnIndex, std::string> _columnNameMap;
    Ui::FileExplorer ui;
    QStandardItemModel _itemModel;
    SshWorker _sshWorkerThread;
};

const QString QSshFileExplorerWidget::STYLE_TREE_VIEW =
    R"(
QTreeView::item {
    padding: 5px;
})";

const std::map<QSshFileExplorerWidget::eColumnIndex, std::string> QSshFileExplorerWidget::_columnNameMap = {
    {eColumnIndex::NAME, "Name"},
    {eColumnIndex::TYPE, "Type"},
    {eColumnIndex::LAST_MODIFIED, "Last modified"}};
