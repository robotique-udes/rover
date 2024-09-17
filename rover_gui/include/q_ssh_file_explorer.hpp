#pragma once

#include <QWidget>
#include <QStandardItemModel>
#include <QTreeView>
#include <QHeaderView>

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
    QSshFileExplorer(QWidget *parent_, QTreeView *treeView_) : roverFileExplorer(new QStandardItemModel(parent_))
    {
        assert(treeView_ != NULL);
        _treeView = treeView_;

        _treeView->setModel(roverFileExplorer);
        roverFileExplorer->setHorizontalHeaderLabels({"Name", "Type", "Last modified"});

        _treeView->header()->setStretchLastSection(false);
        _treeView->header()->setSectionResizeMode((uint8_t)eColumnIndex::NAME, QHeaderView::Stretch);

        _treeView->setColumnWidth((uint8_t)eColumnIndex::TYPE, 50);
        _treeView->header()->setSectionResizeMode((uint8_t)eColumnIndex::TYPE, QHeaderView::Fixed);

        _treeView->setColumnWidth((uint8_t)eColumnIndex::LAST_MODIFIED, 120);
        _treeView->header()->setSectionResizeMode((uint8_t)eColumnIndex::LAST_MODIFIED, QHeaderView::Fixed);

        for (uint8_t i = 0; i < 4; i++)
        {
            QStandardItem *itemName = new QStandardItem();
            itemName->setCheckable(false);
            itemName->setEditable(false);
            itemName->setIcon(_treeView->style()->standardIcon(QStyle::SP_DirIcon));
            itemName->setText(QString(("Item number " + std::to_string(i)).c_str()));

            QStandardItem *itemType = new QStandardItem();
            itemType->setCheckable(false);
            itemType->setEditable(false);
            itemType->setText("Folder");

            QStandardItem *itemModified = new QStandardItem();
            itemModified->setCheckable(false);
            itemModified->setEditable(false);
            itemModified->setText("2024-09-17");
            itemModified->setTextAlignment(Qt::AlignRight);

            roverFileExplorer->appendRow({itemName, itemType, itemModified});
        }
    }

private:
    QTreeView *_treeView;
    QStandardItemModel *roverFileExplorer;
};
