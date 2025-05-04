#ifndef __QDEVICESTATUS_HPP__
#define __QDEVICESTATUS_HPP__

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/can_device_status.hpp"
#include "rover_can2/src/rover_can2/constant.hpp"
#include "rover_lib2/src/rover_lib2/helpers/macros.hpp"

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

  protected:
    int heightForWidth(int width_) const override;

  private:
    void callbackDeviceInfos(const rover_msgs::msg::CanDeviceStatus& msg_);
    void setStatusReport(RoverCan2::Constant::eDeviceId id_);
    QString setErrorMsg(uint8_t errorCode_);

    std::shared_ptr<rclcpp::Node> _node;
    Ui::DeviceStatus _ui;
    rclcpp::Subscription<rover_msgs::msg::CanDeviceStatus>::SharedPtr _sub_deviceStatus;
    std::unordered_map<uint16_t, rover_msgs::msg::CanDeviceStatus> _deviceStatusInfo;
};

#endif  // __QDEVICESTATUS_HPP__