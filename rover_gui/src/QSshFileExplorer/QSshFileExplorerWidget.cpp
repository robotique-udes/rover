#include "QSshFileExplorerWidget.hpp"

#include <rclcpp/rclcpp.hpp>

#include "QTreeViewExplorer.hpp"

#include <QApplication>
#include <QClipboard>
#include <QDesktopServices>
#include <QDir>
#include <QUrl>

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
    connect(_ui.pb_cancelCurrentTask, &QPushButton::clicked, this, [this]() { this->_sshWorkerThread.cancelCurrentTasks(); });
    connect(_ui.pb_cancelAllTasks, &QPushButton::clicked, this, [this]() { this->_sshWorkerThread.cancelAllTasks(); });
    connect(&_refreshKeybind, &QShortcut::activated, this, &QSshFileExplorerWidget::refreshItems);
    connect(&_sshWorkerThread, &QSshWorker::newProgressBarUpdate, this, &QSshFileExplorerWidget::updateProgressBar);
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
    _ui.tv_fileExplorer->setFocusPolicy(Qt::NoFocus);
}

void QSshFileExplorerWidget::initContextMenu(void)
{
    _contextMenu.addAction(&_actionMenuOpen);
    _contextMenu.addSeparator();
    _contextMenu.addAction(&_actionMenuTransfer);

    connect(&_actionMenuOpen, &QAction::triggered, this, &QSshFileExplorerWidget::handleActionMenuOpen);
    connect(&_actionMenuTransfer, &QAction::triggered, this, &QSshFileExplorerWidget::handleActionMenuTransfer);
}

void QSshFileExplorerWidget::linkFriend(QSshFileExplorerWidget* friend_)
{
    assert(friend_ != nullptr);
    _friend = friend_;

    this->linkFriendTreeView(_friend->getUI().tv_fileExplorer);
}

void QSshFileExplorerWidget::linkFriendTreeView(QTreeViewExplorer* treeViewfriend_)
{
    assert(treeViewfriend_ != nullptr);
    connect(treeViewfriend_,
            &QTreeViewExplorer::selectionTriggered,
            this,
            &QSshFileExplorerWidget::handleFriendSelectionTriggered);
}

void QSshFileExplorerWidget::refreshItems(void)
{
    this->refreshItemsNewPath(_ui.le_path->text().toStdString());
}

void QSshFileExplorerWidget::refreshItemsNewPath(const std::string& newPath_)
{
    _sshWorkerThread.refreshStructure(_ui.le_user->text().toStdString(),
                                      _ui.le_hostIP->text().toStdString(),
                                      _ui.le_path->text().toStdString(),
                                      newPath_);
}

void QSshFileExplorerWidget::handleNewStructure(void)
{
    _ui.le_path->setText(_sshWorkerThread.getPathStructure().c_str());

    _itemModel.removeRows(0, _itemModel.rowCount());
    std::vector<QFileItem> files = _sshWorkerThread.getFileStructure();
    bool showHiddenFiles = _ui.cb_showHiddenFile->isChecked();

    for (auto& it : files)
    {
        it.addItemToModel(_itemModel, showHiddenFiles);
    }
}

void QSshFileExplorerWidget::handleItemDoubleClick(const QModelIndex& /*index_*/)
{
    this->handleActionMenuOpen();
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

void QSshFileExplorerWidget::handleActionMenuOpen(void)
{
    QModelIndexList selectedItem = _ui.tv_fileExplorer->selectionModel()->selectedIndexes();

    std::string currentPath = _ui.le_path->text().toStdString();
    for (size_t i = 0u; i < static_cast<size_t>(selectedItem.size()); i++)
    {
        if (selectedItem[i].column() == static_cast<int>(eColumnIndex::NAME)
            && (i + 1u < static_cast<size_t>(selectedItem.size()))
            && selectedItem[i + 1u].column() == static_cast<int>(eColumnIndex::TYPE))
        {
            std::string selectedItemPath = currentPath + "/" + selectedItem[i].data().toString().toStdString();
            std::string cleanItemPath = QDir::cleanPath(selectedItemPath.c_str()).toStdString();

            // Directory
            if (cleanItemPath != "" && selectedItem[i + 1].data().toString().toStdString() == "")
            {
                this->refreshItemsNewPath(cleanItemPath.c_str());
            }
            // File element
            else if (cleanItemPath != "" && selectedItem[i + 1].data().toString().toStdString() != "")
            {
                _sshWorkerThread.openFile(_ui.le_user->text().toStdString(), _ui.le_hostIP->text().toStdString(), cleanItemPath);
            }
        }
    }
}

void QSshFileExplorerWidget::handleActionMenuTransfer(void)
{
    if (!_friend)
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error transfering file, no valid friend QSshFileExplorer linked");
    }
    else
    {
        QModelIndexList selectedItem = _ui.tv_fileExplorer->selectionModel()->selectedIndexes();

        for (size_t i = 0u; i < static_cast<size_t>(selectedItem.size()); i++)
        {
            if (selectedItem[i].column() == static_cast<int>(eColumnIndex::NAME))
            {
                std::string fileName = selectedItem[i].data().toString().toStdString();

                std::string ownerUser = _ui.le_user->text().toStdString();
                std::string ownerHostname = _ui.le_hostIP->text().toStdString();
                std::string ownerFolderPath = _ui.le_path->text().toStdString();

                std::string receiverUsername_ = _friend->getUI().le_user->text().toStdString();
                std::string receiverHostname_ = _friend->getUI().le_hostIP->text().toStdString();
                std::string receiverFolderPath_ = _friend->getUI().le_path->text().toStdString();

                _sshWorkerThread.transferFile(fileName,
                                              ownerUser,
                                              ownerHostname,
                                              ownerFolderPath,
                                              receiverUsername_,
                                              receiverHostname_,
                                              receiverFolderPath_);

                this->refreshItems();
                _friend->refreshItems();
            }
        }
    }
}

void QSshFileExplorerWidget::updateProgressBar(std::string taskDescription_, float progressPercent_)
{
    int progressBarValue = (static_cast<int>(round(_ui.progressBar->maximum() * progressPercent_ / 100.0f)));
    _ui.progressBar->setValue(progressBarValue);

    std::string progressBarText = std::string("    ") + std::to_string(_sshWorkerThread.getTaskNb()) + " tasks remaining.";
    if (taskDescription_ != "")
    {
        progressBarText += " " + taskDescription_;
    }

    _ui.progressBar->setFormat(progressBarText.c_str());
}

const Ui::FileExplorer& QSshFileExplorerWidget::getUI(void) const
{
    return _ui;
}
