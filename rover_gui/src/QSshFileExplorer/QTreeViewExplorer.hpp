#ifndef __QTREE_VIEW_EXPLORER_HPP__
#define __QTREE_VIEW_EXPLORER_HPP__

#include <QApplication>
#include <QMouseEvent>
#include <QTreeView>

#include "rclcpp/rclcpp.hpp"

class QTreeViewExplorer : public QTreeView
{
    Q_OBJECT

  public:
    QTreeViewExplorer(QWidget* parent_): QTreeView(parent_)
    {
        RCLCPP_INFO(rclcpp::get_logger("GUI"), "Overwrite Loaded");
    }

    virtual ~QTreeViewExplorer(){};

    void removeCurrentSelection(void)
    {
        this->selectionModel()->clearSelection();
    }

  signals:
    void selectionTriggered(void);

  private:
    void mousePressEvent(QMouseEvent* event) override
    {
        QModelIndex index = indexAt(event->pos());
        this->handleSelection(index);
    }

    void mouseDoubleClickEvent(QMouseEvent* event) override
    {
        QModelIndex index = indexAt(event->pos());

        if (index.isValid())
        {
            emit this->doubleClicked(index);
        }
    }

    void handleSelection(const QModelIndex& index_)
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
};

#endif  // __QTREE_VIEW_EXPLORER_HPP__
