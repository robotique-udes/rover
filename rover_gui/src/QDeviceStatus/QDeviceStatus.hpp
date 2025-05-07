#ifndef __QDEVICESTATUS_HPP__
#define __QDEVICESTATUS_HPP__

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/can_device_status.hpp"
#include "rover_can2/constant.hpp"
#include "rover_lib2/helpers/macros.hpp"

// QT
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <QStyle>
#include "Global/Constant/StyleSheet.hpp"
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
    void setStatusReport(uint16_t id_);
    QString setErrorMsg(uint8_t errorCode_);
    void updateDeviceButtonColor(QPushButton* button_, const rover_msgs::msg::CanDeviceStatus& deviceStatus_);

    std::shared_ptr<rclcpp::Node> _node;
    Ui::DeviceStatus _ui;
    rclcpp::Subscription<rover_msgs::msg::CanDeviceStatus>::SharedPtr _sub_deviceStatus;
    std::unordered_map<uint16_t, rover_msgs::msg::CanDeviceStatus> _deviceStatusInfo;
    uint16_t _currentStatusID = 0U;

    QMap<uint16_t, QPushButton*> _deviceButtons;
};

#endif  // __QDEVICESTATUS_HPP__