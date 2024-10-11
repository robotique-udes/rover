#pragma once

#include <QDateTime>
#include <QHeaderView>
#include <QInputDialog>
#include <QMessageBox>
#include <QTreeView>

#include "QSshFileExplorer/QFileItem.hpp"
#include "QSshFileExplorer/SshHandler.hpp"

#include "UI_FileExplorer.h"

class QSshFileExplorerWidget : public QWidget
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
    }

    ~QSshFileExplorerWidget() {}

    bool refreshItems(void)
    {
        _itemModel.removeRows(1, _itemModel.rowCount());
        _itemModel.removeRows(1, _itemModel.rowCount());

        std::list<QFileItem> items;
        this->insertGoBackItem(items);

        std::vector<QFileItem> files;
        SshHandler::retrieveFolderStructure("phil", "localhost", "/", files);

        for (auto& it : files)
        {
            it.addItemToModel(_itemModel);
        }

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

  private:
    Ui::FileExplorer ui;
    QStandardItemModel _itemModel;
    std::string _currentPath = "/";
};

const QString QSshFileExplorerWidget::STYLE_TREE_VIEW =
    R"(
        QTreeView::item {
            padding: 5px;
    })";
