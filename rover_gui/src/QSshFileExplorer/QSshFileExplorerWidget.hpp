#pragma once

#include <QApplication>
#include <QClipboard>
#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QFileInfo>
#include <QHeaderView>
#include <QInputDialog>
#include <QMessageBox>
#include <QShortcut>
#include <QTreeView>
#include <QUrl>

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
    QSshFileExplorerWidget(std::string user_ = "", std::string host_ = "", std::string path_ = "", QWidget* parent = nullptr):
        QWidget(parent)
    {
        ui.setupUi(this);

        ui.le_user->setText(user_.c_str());
        ui.le_hostIP->setText(host_.c_str());
        ui.le_path->setText(path_.c_str());

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

        connect(ui.pb_pathCopy,
                &QPushButton::clicked,
                this,
                [this]() { QApplication::clipboard()->setText(ui.le_path->text()); });
        connect(ui.cb_showHiddenFile, &QCheckBox::stateChanged, this, &QSshFileExplorerWidget::refreshItems);
        connect(ui.pb_refresh, &QPushButton::clicked, this, &QSshFileExplorerWidget::refreshItems);
        connect(&_refreshKeybind, &QShortcut::activated, this, &QSshFileExplorerWidget::refreshItems);
        connect(&_sshWorkerThread, &SshWorker::newStructureReady, this, &QSshFileExplorerWidget::handleNewStructure);
        connect(ui.tv_fileExplorer, &QTreeView::doubleClicked, this, &QSshFileExplorerWidget::handleItemDoubleClicked);

        _sshWorkerThread.start();
    }

    ~QSshFileExplorerWidget() {}

    void refreshItems(void)
    {
        _sshWorkerThread.refreshStructure(ui.le_user->text().toStdString(),
                                          ui.le_hostIP->text().toStdString(),
                                          ui.le_path->text().toStdString());
    }

    void handleNewStructure(void)
    {
        _itemModel.removeRows(0, _itemModel.rowCount());

        std::vector<QFileItem> files = _sshWorkerThread.getStructure();
        bool showHiddenFiles = ui.cb_showHiddenFile->isChecked();

        for (auto& it : files)
        {
            it.addItemToModel(_itemModel, showHiddenFiles);
        }
    }

    void handleItemDoubleClicked(const QModelIndex& index_)
    {
        if (!index_.isValid())
        {
            return;
        }

        QString currentPath = ui.le_path->text();
        QString newPath = currentPath.append("/").append(_itemModel.item(index_.row(), (size_t)eColumnIndex::NAME)->text());
        newPath = QDir(newPath).canonicalPath();

        if (_itemModel.item(index_.row(), (size_t)eColumnIndex::TYPE)->text() != "")
        {
            QFileInfo fileInfo(newPath);

            if (fileInfo.exists() && fileInfo.isFile())
            {
                QApplication::setOverrideCursor(Qt::WaitCursor);
                QDesktopServices::openUrl(QUrl::fromLocalFile(newPath));
                QApplication::restoreOverrideCursor();
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("GUI"), "Openning file not supported yet");
            }

            return;
        }

        ui.le_path->setText(newPath);
        this->refreshItems();
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
    QShortcut _refreshKeybind = QShortcut(QKeySequence("Return"), this);
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
