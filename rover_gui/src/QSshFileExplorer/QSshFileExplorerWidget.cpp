#include "QSshFileExplorerWidget.hpp"

#include <rclcpp/rclcpp.hpp>

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
    ui.setupUi(this);

    std::string path = path_;
    if (path == "")
    {
        path = "/home/" + user_;
    }
    ui.le_user->setText(user_.c_str());
    ui.le_hostIP->setText(host_.c_str());
    ui.le_path->setText(path.c_str());

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

    connect(ui.pb_pathCopy, &QPushButton::clicked, this, [this]() { QApplication::clipboard()->setText(ui.le_path->text()); });
    connect(ui.cb_showHiddenFile, &QCheckBox::stateChanged, this, &QSshFileExplorerWidget::refreshItems);
    connect(ui.pb_refresh, &QPushButton::clicked, this, &QSshFileExplorerWidget::refreshItems);
    connect(&_refreshKeybind, &QShortcut::activated, this, &QSshFileExplorerWidget::refreshItems);
    connect(&_sshWorkerThread, &QSshWorker::newStructureReady, this, &QSshFileExplorerWidget::handleNewStructure);
    connect(ui.tv_fileExplorer, &QTreeView::doubleClicked, this, &QSshFileExplorerWidget::handleItemDoubleClicked);

    _sshWorkerThread.start();
}

QSshFileExplorerWidget::~QSshFileExplorerWidget() {}

void QSshFileExplorerWidget::refreshItems(void)
{
    _sshWorkerThread.refreshStructure(ui.le_user->text().toStdString(),
                                      ui.le_hostIP->text().toStdString(),
                                      ui.le_path->text().toStdString());
}

void QSshFileExplorerWidget::handleNewStructure(void)
{
    _itemModel.removeRows(0, _itemModel.rowCount());

    std::vector<QFileItem> files = _sshWorkerThread.getStructure();
    bool showHiddenFiles = ui.cb_showHiddenFile->isChecked();

    for (auto& it : files)
    {
        it.addItemToModel(_itemModel, showHiddenFiles);
    }
}

void QSshFileExplorerWidget::handleItemDoubleClicked(const QModelIndex& index_)
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

const Ui::FileExplorer& QSshFileExplorerWidget::getUI(void) const
{
    return ui;
}
