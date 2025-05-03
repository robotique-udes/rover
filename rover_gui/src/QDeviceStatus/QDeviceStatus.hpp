#ifndef __QDEVICESTATUS_HPP__

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/gps.hpp"

// QT
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include "UI_DeviceStatus.h"

class DeviceStatus : public QWidget
   {
   public:
   DeviceStatus(QWidget* parent_): QWidget(parent)
   	{
   		_ui.setupUi(this);
   	}
   	~DeviceStatus();

   private:
   	Ui::DeviceStatus _ui;
   };

#endif // __QDEVICESTATUS_HPP__