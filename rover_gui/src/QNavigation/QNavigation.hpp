#ifndef QNAVIGATION_HPP
#define QNAVIGATION_HPP

#include "rclcpp/rclcpp.hpp"

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

class QNavigation : public QWidget
{
    Q_OBJECT

  public:
    QNavigation(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
        QWidget(parent_),
        _node(guiNode_),
        _navigationLayout(this),
        _navigationLabel(this)
    {
        _navigationLabel.setAlignment(Qt::AlignCenter);
        _navigationLayout.addWidget(&_navigationLabel);

        this->setLayout(&_navigationLayout);
    }

  private:
    std::shared_ptr<rclcpp::Node> _node;
    QGridLayout _navigationLayout;
    QLabel _navigationLabel;
};

#endif  // QNAVIGATION_HPP
