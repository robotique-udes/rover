#include "QSshFileExplorerWidget.hpp"

#include <rclcpp/rclcpp.hpp>

#include <QApplication>
#include <QClipboard>
#include <QDesktopServices>
#include <QDir>
#include <QUrl>

#include "QTreeViewExplorer.hpp"

const std::map<QSshFileExplorerWidget::eColumnIndex, std::string> QSshFileExplorerWidget::_columnNameMap
    = {{eColumnIndex::NAME, "Name"}, {eColumnIndex::TYPE, "Type"}, {eColumnIndex::LAST_MODIFIED, "Last modified"}};

QSshFileExplorerWidget::QSshFileExplorerWidget(const std::string& user_,
                                               const std::string& host_,
                                               const std::string& path_,
                                               QWidget* parent_):
    QWidget(parent_)
{
    _ui.setupUi(this);

    std::string path = path_;
    if (path == "")
    {
        path = "/home/" + user_;
    }
    _ui.le_user->setText(user_.c_str());
    _ui.le_hostIP->setText(host_.c_str());
    _ui.le_path->setText(path.c_str());

    this->initTreeView();
    this->initContextMenu();

    this->refreshItems();

    connect(_ui.pb_pathCopy, &QPushButton::clicked, this, [this]() { QApplication::clipboard()->setText(_ui.le_path->text()); });
    connect(_ui.cb_showHiddenFile, &QCheckBox::stateChanged, this, &QSshFileExplorerWidget::refreshItems);
    connect(_ui.pb_refresh, &QPushButton::clicked, this, &QSshFileExplorerWidget::refreshItems);
    connect(&_refreshKeybind, &QShortcut::activated, this, &QSshFileExplorerWidget::refreshItems);
    connect(&_sshWorkerThread, &QSshWorker::newStructureReady, this, &QSshFileExplorerWidget::handleNewStructure);

    connect(_ui.tv_fileExplorer, &QTreeView::doubleClicked, this, &QSshFileExplorerWidget::handleItemDoubleClick);
    connect(_ui.tv_fileExplorer, &QTreeView::customContextMenuRequested, this, &QSshFileExplorerWidget::handleRightClick);

    _sshWorkerThread.start();
}

QSshFileExplorerWidget::~QSshFileExplorerWidget() {}

void QSshFileExplorerWidget::initTreeView(void)
{
    _ui.tv_fileExplorer->setModel(&_itemModel);
    QFont fountSize;
    fountSize.setPointSize(11);
    _ui.tv_fileExplorer->setFont(fountSize);
    QString currentStyle = _ui.tv_fileExplorer->styleSheet();
    currentStyle.append(STYLE_TREE_VIEW);
    _ui.tv_fileExplorer->setStyleSheet(currentStyle);

    _itemModel.setHorizontalHeaderLabels({"Name", "Type", "Last modified"});

    _ui.tv_fileExplorer->header()->setStretchLastSection(false);
    _ui.tv_fileExplorer->header()->setSectionResizeMode((uint8_t)eColumnIndex::NAME, QHeaderView::Stretch);
    _ui.tv_fileExplorer->setColumnWidth((uint8_t)eColumnIndex::TYPE, 50);
    _ui.tv_fileExplorer->header()->setSectionResizeMode((uint8_t)eColumnIndex::TYPE, QHeaderView::Fixed);
    _ui.tv_fileExplorer->setColumnWidth((uint8_t)eColumnIndex::LAST_MODIFIED, 150);
    _ui.tv_fileExplorer->header()->setSectionResizeMode((uint8_t)eColumnIndex::LAST_MODIFIED, QHeaderView::Fixed);
    _ui.tv_fileExplorer->setContextMenuPolicy(Qt::CustomContextMenu);
}

void QSshFileExplorerWidget::initContextMenu(void)
{
    _contextMenu.addAction(&_actionMenuOpen);
    _contextMenu.addSeparator();
    _contextMenu.addAction(&_actionMenuRename);
    _contextMenu.addSeparator();
    _contextMenu.addAction(&_actionMenuCut);
    _contextMenu.addAction(&_actionMenuPaste);
    _contextMenu.addSeparator();
    _contextMenu.addAction(&_actionMenuDelete);

    connect(&_actionMenuOpen, &QAction::triggered, this, &QSshFileExplorerWidget::handleActionMenuOpen);
    connect(&_actionMenuCut, &QAction::triggered, this, &QSshFileExplorerWidget::handleActionMenuCut);
    connect(&_actionMenuPaste, &QAction::triggered, this, &QSshFileExplorerWidget::handleActionMenuPaste);
    connect(&_actionMenuDelete, &QAction::triggered, this, &QSshFileExplorerWidget::handleActionMenuDelete);
    connect(&_actionMenuRename, &QAction::triggered, this, &QSshFileExplorerWidget::handleActionMenuRename);
}

void QSshFileExplorerWidget::linkFriendTreeView(QTreeViewExplorer* treeViewfriend_)
{
    if (treeViewfriend_)
    {
        // clang-format off
        connect(treeViewfriend_, &QTreeViewExplorer::selectionTriggered, this, &QSshFileExplorerWidget::handleFriendSelectionTriggered);
        // clang-format on
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Specified friend is nullptr aborting");
        abort();
    }
}

void QSshFileExplorerWidget::refreshItems(void)
{
    _sshWorkerThread.refreshStructure(_ui.le_user->text().toStdString(),
                                      _ui.le_hostIP->text().toStdString(),
                                      _ui.le_path->text().toStdString());
}

void QSshFileExplorerWidget::handleNewStructure(void)
{
    _itemModel.removeRows(0, _itemModel.rowCount());

    std::vector<QFileItem> files = _sshWorkerThread.getStructure();
    bool showHiddenFiles = _ui.cb_showHiddenFile->isChecked();

    for (auto& it : files)
    {
        it.addItemToModel(_itemModel, showHiddenFiles);
    }
}

void QSshFileExplorerWidget::handleItemDoubleClick(const QModelIndex& index_)
{
    if (!index_.isValid())
    {
        return;
    }

    QString currentPath = _ui.le_path->text();
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

    _ui.le_path->setText(newPath);
    this->refreshItems();
}

void QSshFileExplorerWidget::handleFriendSelectionTriggered(void)
{
    if (_ui.tv_fileExplorer)
    {
        _ui.tv_fileExplorer->removeCurrentSelection();
    }
}

void QSshFileExplorerWidget::handleRightClick(const QPoint& pos_)
{
    if (_ui.tv_fileExplorer)
    {
        _contextMenuIndex = _ui.tv_fileExplorer->indexAt(pos_);
        _contextMenu.exec(_ui.tv_fileExplorer->viewport()->mapToGlobal(pos_));
    }
}

void QSshFileExplorerWidget::handleActionMenuOpen(void) {}

void QSshFileExplorerWidget::handleActionMenuRename(void) {}

void QSshFileExplorerWidget::handleActionMenuCut(void) {}

void QSshFileExplorerWidget::handleActionMenuPaste(void) {}

void QSshFileExplorerWidget::handleActionMenuDelete(void) {}

const Ui::FileExplorer& QSshFileExplorerWidget::getUI(void) const
{
    return _ui;
}
