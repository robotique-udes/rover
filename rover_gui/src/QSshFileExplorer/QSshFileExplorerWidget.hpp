#ifndef __QSSH_FILE_EXPLORER_WIDGET_HPP__
#define __QSSH_FILE_EXPLORER_WIDGET_HPP__

#include <QShortcut>

#include "QFileItem.hpp"
#include "QSshWorker.hpp"

#include "UI_FileExplorer.h"

class QSshFileExplorerWidget : public QWidget
{
    enum class eColumnIndex : uint8_t
    {
        NAME = 0,
        TYPE,
        LAST_MODIFIED,
        eLAST
    };

    static const QString STYLE_TREE_VIEW;

  public:
    QSshFileExplorerWidget(std::string user_ = "", std::string host_ = "", std::string path_ = "", QWidget* parent = nullptr);
    ~QSshFileExplorerWidget();

    void refreshItems(void);
    void handleNewStructure(void);
    void handleItemDoubleClicked(const QModelIndex& index_);
    const Ui::FileExplorer& getUI(void);

  private:
    static const std::map<eColumnIndex, std::string> _columnNameMap;
    Ui::FileExplorer ui;
    QStandardItemModel _itemModel;
    QSshWorker _sshWorkerThread;
    QShortcut _refreshKeybind = QShortcut(QKeySequence("Return"), this);
};

#endif // __QSSH_FILE_EXPLORER_WIDGET_HPP__
