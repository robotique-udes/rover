#ifndef QBMSDATA_HPP
#define QBMSDATA_HPP

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/can_device_status.hpp>
#include <rover_msgs/srv/empty.hpp>
#include <rover_can2/constant.hpp>
#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>

// QT
#include <QtWidgets/QGridLayout>
#include "UI_BmsData.h"


class QBmsData : public QWidget
{
    Q_OBJECT

    public:
        QBmsData(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);


    private:
        std::shared_ptr<rclcpp::Node> _node;
        Ui::DataLogger _ui;


};

#endif