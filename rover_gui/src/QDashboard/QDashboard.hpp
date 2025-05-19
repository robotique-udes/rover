#ifndef QDASHBOARD_HPP
#define QDASHBOARD_HPP

#include "rclcpp/rclcpp.hpp"
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>

#include "QExample.hpp"
#include "QArbitration.hpp"


class QDashboard : public QWidget
{
    Q_OBJECT

  public:
    QDashboard(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
        QWidget(parent_),
        _node(guiNode_),
        _dashboardLayout(this),
        _exampleWidget(guiNode_, this),
        _arbitrationWidget(guiNode_, this)
    {
        this->setLayout(&_dashboardLayout);

        // Add your dashboard widget here
        _dashboardLayout.addWidget(&_exampleWidget);
        _dashboardLayout.addWidget(&_arbitrationWidget);


    }

  private:
    std::shared_ptr<rclcpp::Node> _node;

    QGridLayout _dashboardLayout;
    QExample _exampleWidget;
    QArbitration _arbitrationWidget;
};

#endif  // QDASHBOARD_HPP
