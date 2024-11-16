#ifndef __QSSH_FILE_EXPLORER_WIDGET_HPP__
#define __QSSH_FILE_EXPLORER_WIDGET_HPP__

#include <QMenu>
#include <QShortcut>

#include "QFileItem.hpp"
#include "Worker/QSshWorker.hpp"

#include "UI_FileExplorer.h"

class QSshFileExplorerWidget : public QWidget
{
    static constexpr const char* STYLE_TREE_VIEW = R"(
QTreeView::item {
    padding: 5px;
})";

    enum class eColumnIndex : uint8_t
    {
        NAME = 0,
        TYPE,
        LAST_MODIFIED,
        eLAST
    };

  public:
    QSshFileExplorerWidget(const std::string& user_, const std::string& host_, const std::string& path_, QWidget* parent_);
    ~QSshFileExplorerWidget();

    const Ui::FileExplorer& getUI(void) const;

    void linkFriendTreeView(QTreeViewExplorer* treeViewfriend_);

  private:
    static const std::map<eColumnIndex, std::string> _columnNameMap;
    Ui::FileExplorer _ui;
    QStandardItemModel _itemModel;
    QSshWorker _sshWorkerThread;
    QShortcut _refreshKeybind = QShortcut(QKeySequence("Return"), this);

    QMenu _contextMenu = QMenu(this);
    QAction _actionMenuOpen = QAction("Open", this);
    QAction _actionMenuRename = QAction("Rename", this);
    QAction _actionMenuCut = QAction("Cut", this);
    QAction _actionMenuPaste = QAction("Paste", this);
    QAction _actionMenuDelete = QAction("Delete", this);
    QModelIndex _contextMenuIndex = QModelIndex();

    void initTreeView(void);
    void initContextMenu(void);

  private slots:
    void refreshItems(void);

    void handleFriendSelectionTriggered(void);
    void handleNewStructure(void);
    void handleItemDoubleClick(const QModelIndex& index_);
    void handleLeftClick(const QModelIndex& index_);
    void handleRightClick(const QPoint& pos_);
    void handleActionMenuOpen(void);
    void handleActionMenuRename(void);
    void handleActionMenuCut(void);
    void handleActionMenuPaste(void);
    void handleActionMenuDelete(void);

    void updateProgressBar(std::string taskDescription_, float progressPercent_);
};

#endif  // __QSSH_FILE_EXPLORER_WIDGET_HPP__
