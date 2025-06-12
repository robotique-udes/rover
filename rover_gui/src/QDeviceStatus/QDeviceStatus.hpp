#ifndef QDEVICESTATUS_QDEVICESTATUS_HPP
#define QDEVICESTATUS_QDEVICESTATUS_HPP

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/can_device_status.hpp>
#include <rover_msgs/srv/empty.hpp>
#include <rover_can2/constant.hpp>
#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>

// QT
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include "UI_DeviceStatus.h"
#include "Worker/QStatusWorker.hpp"

#include <unordered_map>

class QDeviceStatus : public QWidget
{
    Q_OBJECT

  public:
    QDeviceStatus(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);
    ~QDeviceStatus() = default;

  private:
    void callbackDeviceInfos(const rover_msgs::msg::CanDeviceStatus& msg_);
    void updateDeviceInfo();
    void updateDeviceColor(RoverCan2::Constant::eDeviceId deviceID_, const rover_msgs::msg::CanDeviceStatus& msg_);
    void setStatusReport(RoverCan2::Constant::eDeviceId id_);
    void updateRebootCounter(RoverCan2::Constant::eDeviceId deviceID_);
    void setDefaultStyle();
    std::string getDeviceName(RoverCan2::Constant::eDeviceId deviceID_);

  private slots:

    void onRequestDeviceStatusSuccessful(bool success_, const std::string& response_);

  private:
    std::shared_ptr<rclcpp::Node> _node;
    Ui::DeviceStatus _ui;
    QLabel* _imageLabel;
    QPixmap _pixmap;

    rclcpp::Subscription<rover_msgs::msg::CanDeviceStatus>::SharedPtr _sub_deviceStatus;
    rclcpp::Client<rover_msgs::srv::Empty>::SharedPtr _client_requestErrorStatus;

    std::unordered_map<RoverCan2::Constant::eDeviceId, uint16_t> _deviceMessageCount;
    std::unordered_map<RoverCan2::Constant::eDeviceId, uint16_t> _numberOfDeviceReboots;
    std::unordered_map<RoverCan2::Constant::eDeviceId, uint16_t> _oldDeviceReboots;
    std::unordered_map<RoverCan2::Constant::eDeviceId, uint16_t> _numberOfDeviceRebootsFromButton;

    std::unordered_map<RoverCan2::Constant::eDeviceId, QWidget*> _deviceInfo;
    std::unordered_map<RoverCan2::Constant::eDeviceId, QLabel*> _deviceReboot;

    uint16_t _numberOfCalls = 0U;

    QStatusWorker _QStatusWorker;
};

#endif  // QDEVICESTATUS_QDEVICESTATUS_HPP