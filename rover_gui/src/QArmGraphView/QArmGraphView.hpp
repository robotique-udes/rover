#ifndef QARMGRAPHVIEW_HPP
#define QARMGRAPHVIEW_HPP

#include "UI_ArmGraphicView.h"


// ROS
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/arm_msg.hpp>
#include <rover_can2/constant.hpp>
#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>

// QT
#include <QGraphicsView>
#include <QGraphicsScene>


class QArmGraphView : public QWidget
{

    Q_OBJECT

    public:
        QArmGraphView(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);

    signals:
        void callbackArmGraphView(rover_msgs::msg::ArmMsg& msg_);

    private slots:
        void oncallbackArmGraphView(const rover_msgs::msg::ArmMsg& msg_);

    private:
        void initializeWidget(void);

        std::shared_ptr<rclcpp::Node> _node;
        Ui::Form _ui;
        rclcpp::Subscription<rover_msgs::msg::ArmMsg>::SharedPtr _sub_armMsg;
        QWidget* _armViewWidget;
        QGraphicsScene* _scene;

};

#endif