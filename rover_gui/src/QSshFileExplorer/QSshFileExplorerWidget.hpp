#ifndef __QSSH_FILE_EXPLORER_WIDGET_HPP__
#define __QSSH_FILE_EXPLORER_WIDGET_HPP__

#include <QShortcut>

#include "QFileItem.hpp"
#include "QSshWorker.hpp"

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

  private:
    static const std::map<eColumnIndex, std::string> _columnNameMap;
    Ui::FileExplorer ui;
    QStandardItemModel _itemModel;
    QSshWorker _sshWorkerThread;
    QShortcut _refreshKeybind = QShortcut(QKeySequence("Return"), this);

  private slots:
    void refreshItems(void);
    void handleNewStructure(void);
    void handleItemDoubleClicked(const QModelIndex& index_);
};

#endif  // __QSSH_FILE_EXPLORER_WIDGET_HPP__
