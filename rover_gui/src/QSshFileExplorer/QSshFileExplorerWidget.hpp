#ifndef __QSSH_FILE_EXPLORER_WIDGET_HPP__
#define __QSSH_FILE_EXPLORER_WIDGET_HPP__

#include "UI_FileExplorer.h"

#include "QFileItem.hpp"
#include "Worker/QSshWorker.hpp"

#include <QMenu>
#include <QShortcut>
#include <QStandardItemModel>

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

    /**
     * @brief [WARNING] This method contain runtime assert, for critical application only call at initialisation!
     *
     */
    void linkFriend(QSshFileExplorerWidget* friend_);

  public slots:
    /**
     * @brief Refresh the file tree window from the current informations
     * displayed
     *
     */
    void refreshItems(void);

    /**
     * @brief Refresh the file tree window from the new specified path
     *
     *
     */
    void refreshItemsNewPath(const std::string& newPath_);

  private:
    QSshFileExplorerWidget* _friend = nullptr;

    static const std::map<eColumnIndex, std::string> _columnNameMap;
    Ui::FileExplorer _ui;
    QStandardItemModel _itemModel;
    QSshWorker _sshWorkerThread;
    QShortcut _refreshKeybind = QShortcut(QKeySequence("Return"), this);

    QMenu _contextMenu = QMenu(this);
    QAction _actionMenuOpen = QAction("Open", this);
    QAction _actionMenuTransfer = QAction("Transfer", this);
    QModelIndex _contextMenuIndex = QModelIndex();

    void initTreeView(void);
    void initContextMenu(void);

    /**
     * @brief [WARNING] This method contain runtime assert, for critical application only call at initialisation!
     *
     */
    void linkFriendTreeView(QTreeViewExplorer* treeViewfriend_);

  private slots:

    void handleFriendSelectionTriggered(void);
    void handleNewStructure(void);
    void handleItemDoubleClick(const QModelIndex& index_);
    void handleLeftClick(const QModelIndex& index_);
    void handleRightClick(const QPoint& pos_);
    void handleActionMenuOpen(void);
    void handleActionMenuTransfer(void);

    void updateProgressBar(std::string taskDescription_, float progressPercent_);
};

#endif  // __QSSH_FILE_EXPLORER_WIDGET_HPP__
