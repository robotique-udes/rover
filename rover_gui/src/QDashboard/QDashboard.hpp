#ifndef QDASHBOARD_HPP
#define QDASHBOARD_HPP

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>

#include "QArbitration.hpp"

class QDashboard : public QWidget
{
    Q_OBJECT

  public:
    QDashboard(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
        QWidget(parent_),
        _node(guiNode_),
        _dashboardLayout(this),
        _arbitrationWidget(guiNode_, this),
        _emptyWidget(QWidget(this))
    {
        this->setLayout(&_dashboardLayout);

        // Add your dashboard widget here
        _dashboardLayout.addWidget(&_arbitrationWidget, 1, 0);
        _dashboardLayout.addWidget(&_emptyWidget, 0, 0);
    }

  private:
    std::shared_ptr<rclcpp::Node> _node;

    QGridLayout _dashboardLayout;
    QArbitration _arbitrationWidget;
    QWidget _emptyWidget;
};

#endif  // QDASHBOARD_HPP
