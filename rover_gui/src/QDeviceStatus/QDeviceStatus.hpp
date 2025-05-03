//#ifndef __QDEVICESTATUS_HPP__
#pragma once

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/gps.hpp"

// QT
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include "UI_DeviceStatus.h"

class QDeviceStatus : public QWidget
   {

        Q_OBJECT

   public:
   QDeviceStatus(QWidget* parent_ = nullptr);
   	~QDeviceStatus();

   private:
   	Ui::DeviceStatus _ui;
   };

//#endif // __QDEVICESTATUS_HPP__