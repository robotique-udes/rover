#pragma once

#include <QDateTime>
#include <QHeaderView>
#include <QInputDialog>
#include <QMessageBox>
#include <QTreeView>

#include "QFileItem.hpp"
#include "SshHandler.hpp"

class QSshFileExplorer
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
    QSshFileExplorer(QTreeView& rTreeView_): _rTreeView(rTreeView_)
    {
        _rTreeView.setModel(&_itemModel);
        QFont fountSize;
        fountSize.setPointSize(12);
        _rTreeView.setFont(fountSize);
        QString currentStyle = _rTreeView.styleSheet();
        currentStyle.append(STYLE_TREE_VIEW);
        rTreeView_.setStyleSheet(currentStyle);

        _itemModel.setHorizontalHeaderLabels({"Name", "Type", "Last modified"});

        _rTreeView.header()->setStretchLastSection(false);
        _rTreeView.header()->setSectionResizeMode((uint8_t)eColumnIndex::NAME, QHeaderView::Stretch);

        _rTreeView.setColumnWidth((uint8_t)eColumnIndex::TYPE, 50);
        _rTreeView.header()->setSectionResizeMode((uint8_t)eColumnIndex::TYPE, QHeaderView::Fixed);

        _rTreeView.setColumnWidth((uint8_t)eColumnIndex::LAST_MODIFIED, 150);
        _rTreeView.header()->setSectionResizeMode((uint8_t)eColumnIndex::LAST_MODIFIED, QHeaderView::Fixed);

        this->refreshItems();

        std::vector<QFileItem> files;
        SshHandler::retrieveFolderStructure("phil", "localhost", "/", files);

        for (auto& it : files)
        {
            it.addItemToModel(_itemModel);
        }
    }

    bool refreshItems(void)
    {
        _itemModel.removeRows(1, _itemModel.rowCount());
        _itemModel.removeRows(1, _itemModel.rowCount());

        std::list<QFileItem> items;
        this->insertGoBackItem(items);

#warning TODO: Implement ssh ls
        // for (const auto &[extension, icon] : IconManager::getIconMap())
        // {
        //     // Generate a name for each item (you might want to customize this)
        //     std::string name = extension.empty() ? "Folder" : "File " + extension;

        //     // Create a QFileItem instance and add it to the list
        //     items.push_back(QFileItem(name, extension, "2024-13-17"));
        // }

        for (auto& it : items)
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
    QTreeView& _rTreeView;
    QStandardItemModel _itemModel;
    std::string _currentPath = "/";
};

const QString QSshFileExplorer::STYLE_TREE_VIEW =
    R"(
        QTreeView::item {
            padding: 5px;
    })";
