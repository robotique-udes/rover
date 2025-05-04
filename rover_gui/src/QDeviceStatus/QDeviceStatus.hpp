//#ifndef __QDEVICESTATUS_HPP__
#pragma once

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/can_device_status.hpp"

// QT
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include "UI_DeviceStatus.h"

#include <unordered_map>

class QDeviceStatus : public QWidget
   {

      Q_OBJECT

   public:
      QDeviceStatus(QWidget* parent_ = nullptr);
   	~QDeviceStatus() = default;

   protected:
      int heightForWidth(int width_) const override;

   private:
      void callbackDeviceInfos(const rover_msgs::msg::CanDeviceStatus& msg_);

   	Ui::DeviceStatus _ui;
      rclcpp::Subscription<rover_msgs::msg::CanDeviceStatus>::SharedPtr _sub_deviceStatus;
      std::unordered_map<uint16_t, rover_msgs::msg::CanDeviceStatus> _deviceStatusInfo;
   };

//#endif // __QDEVICESTATUS_HPP__