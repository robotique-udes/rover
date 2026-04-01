#ifndef QBMSDATA_HPP
#define QBMSDATA_HPP

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/bms_data.hpp>
#include <rover_can2/constant.hpp>
#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>

// QT
#include <QtWidgets/QGridLayout>
#include "UI_BmsData.h"
#include "Global/QFlowLayout.hpp"


class QBmsData : public QWidget
{
    Q_OBJECT

    public:
        QBmsData(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);
        void callbackBmsData(const rover_msgs::msg::BmsData& msg_);


    private:
        std::shared_ptr<rclcpp::Node> _node;
        Ui::DataLogger _ui;
        rclcpp::Subscription<rover_msgs::msg::BmsData>::SharedPtr _sub_bmsData;
        std::unique_ptr<QFlowLayout> _layout;


};

#endif