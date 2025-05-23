#ifndef __QDEVICESTATUS_HPP__
#define __QDEVICESTATUS_HPP__

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/can_device_status.hpp"
#include "rover_msgs/srv/empty.hpp"
#include "rover_can2/constant.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/constants.hpp"

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
    void updateDeviceColor(uint16_t deviceID_, const rover_msgs::msg::CanDeviceStatus& deviceStatus_);
    void setStatusReport(uint16_t id_);
    void updateRebootCounter(uint16_t deviceID_);
    void setDefaultStyle();

  private slots:

    void onRequestDeviceStatusSuccessful(bool success_, const std::string status_);

  private:
    std::shared_ptr<rclcpp::Node> _node;
    Ui::DeviceStatus _ui;
    QLabel* _imageLabel;
    QPixmap _pixmap;

    rclcpp::Subscription<rover_msgs::msg::CanDeviceStatus>::SharedPtr _sub_deviceStatus;
    rclcpp::Client<rover_msgs::srv::Empty>::SharedPtr _client_requestErrorStatus;

    std::unordered_map<uint16_t, int16_t> _deviceMessageCount;
    std::unordered_map<uint16_t, uint16_t> _numberOfDeviceReboots;
    std::unordered_map<uint16_t, uint16_t> _oldDeviceReboots;
    std::unordered_map<uint16_t, uint16_t> _numberOfDeviceRebootsFromButton;

    QMap<uint16_t, QLabel*> _deviceInfo;
    QMap<uint16_t, QLabel*> _deviceReboot;

    uint16_t _numberOfCalls = 0U;

    QStatusWorker* _QStatusWorker;
};

#endif  // __QDEVICESTATUS_HPP__