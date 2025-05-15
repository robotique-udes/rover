#ifndef __QDEVICESTATUS_HPP__
#define __QDEVICESTATUS_HPP__

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/can_device_status.hpp"
#include "rover_msgs/srv/empty.hpp"
#include "rover_can2/constant.hpp"
#include "rover_lib2/helpers/macros.hpp"

// QT
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include "UI_DeviceStatus.h"

#include <unordered_map>

class QDeviceStatus : public QWidget
{
    Q_OBJECT

  public:
    QDeviceStatus(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);
    ~QDeviceStatus() = default;

    // protected:
    //   int heightForWidth(int width_) const override;

  private:
    void callbackDeviceInfos(const rover_msgs::msg::CanDeviceStatus& msg_);
    void setStatusReport(uint16_t id_);
    void updateDevicesColor();
    void updateDeviceInfo(std::shared_ptr<rover_msgs::srv::Empty::Request> request_);
    void rebootDevice(uint16_t id_);

    std::shared_ptr<rclcpp::Node> _node;
    Ui::DeviceStatus _ui;

    rclcpp::Subscription<rover_msgs::msg::CanDeviceStatus>::SharedPtr _sub_deviceStatus;
    rclcpp::Client<rover_msgs::srv::Empty>::SharedPtr _client;

    std::unordered_map<uint16_t, rover_msgs::msg::CanDeviceStatus> _deviceStatusInfo;
    QMap<uint16_t, QPushButton*> _deviceButtons;
    QMap<uint16_t, QLabel*> _deviceLabels;
};

#endif  // __QDEVICESTATUS_HPP__