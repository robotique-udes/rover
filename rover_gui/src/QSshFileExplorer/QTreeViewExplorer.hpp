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
    QTreeViewExplorer(QWidget* parent_);
    virtual ~QTreeViewExplorer();

    void removeCurrentSelection(void);

  signals:
    void selectionTriggered(void);

  private:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void handleSelection(const QModelIndex& index_);
};

#endif  // __QTREE_VIEW_EXPLORER_HPP__
