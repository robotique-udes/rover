#include "QTreeViewExplorer.hpp"

QTreeViewExplorer::QTreeViewExplorer(QWidget* parent_): QTreeView(parent_) {}

QTreeViewExplorer::~QTreeViewExplorer(){};

void QTreeViewExplorer::removeCurrentSelection(void)
{
    this->selectionModel()->clearSelection();
}

void QTreeViewExplorer::mousePressEvent(QMouseEvent* event)
{
    QModelIndex index = indexAt(event->pos());
    this->handleSelection(index);
}

void QTreeViewExplorer::mouseDoubleClickEvent(QMouseEvent* event)
{
    QModelIndex index = indexAt(event->pos());

    if (index.isValid())
    {
        emit this->doubleClicked(index);
    }
}

void QTreeViewExplorer::handleSelection(const QModelIndex& index_)
{
    if (index_.isValid())
    {
        if (QApplication::keyboardModifiers() & Qt::ControlModifier)
        {
            this->selectionModel()->select(index_, (QItemSelectionModel::Select | QItemSelectionModel::Rows));
        }
        else
        {
            this->selectionModel()->select(index_, (QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows));
        }
    }

    emit this->selectionTriggered();
}
