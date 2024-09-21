#pragma once

#include <QWidget>
#include <QStandardItemModel>
#include <QTreeView>
#include <QHeaderView>

#include "q_file_item.hpp"

class QSshFileExplorer
{
private:
    enum class eColumnIndex : uint8_t
    {
        NAME,
        TYPE,
        LAST_MODIFIED
    };

public:
    QSshFileExplorer(QTreeView &rTreeView_) : _rTreeView(rTreeView_)
    {
        _rTreeView.setModel(&fileExplorerModel);
        QFont fountSize;
        fountSize.setPointSize(12);
        _rTreeView.setFont(fountSize);

        fileExplorerModel.setHorizontalHeaderLabels({"Name", "Type", "Last modified"});

        _rTreeView.header()->setStretchLastSection(false);
        _rTreeView.header()->setSectionResizeMode((uint8_t)eColumnIndex::NAME, QHeaderView::Stretch);

        _rTreeView.setColumnWidth((uint8_t)eColumnIndex::TYPE, 50);
        _rTreeView.header()->setSectionResizeMode((uint8_t)eColumnIndex::TYPE, QHeaderView::Fixed);

        _rTreeView.setColumnWidth((uint8_t)eColumnIndex::LAST_MODIFIED, 120);
        _rTreeView.header()->setSectionResizeMode((uint8_t)eColumnIndex::LAST_MODIFIED, QHeaderView::Fixed);

        IconManager::initializeIcons();
        this->refreshItems();
    }

    bool refreshItems(void)
    {
        std::list<QFileItem> items;
        this->insertGoBackItem(items);
        // for (uint8_t i = 0; i < 4; i++)
        // {
        //     items.push_back(QFileItem(std::string("Folder " + std::to_string(i)), "", "2024-13-17"));
        // }

        for (const auto &[extension, icon] : IconManager::getIconMap())
        {
            // Generate a name for each item (you might want to customize this)
            std::string name = extension.empty() ? "Folder" : "File " + extension;

            // Create a QFileItem instance and add it to the list
            items.push_back(QFileItem(name, extension, "2024-13-17"));
        }

        for (auto &it : items)
        {
            it.addItemToModel(fileExplorerModel);
        }

        return true;
    }

    void insertGoBackItem(std::list<QFileItem> &items)
    {
        if (items.size() != 0 && items.front().getName() == QFileItemGoBack().getName())
        {
            return;
        }
        items.emplace_front(QFileItemGoBack());
    }

private:
    QTreeView &_rTreeView;
    QStandardItemModel fileExplorerModel;
};
