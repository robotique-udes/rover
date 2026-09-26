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
#include <QLabel>
#include "ui_DeviceStatus.h"
#include "Worker/QStatusWorker.hpp"
#include "Global/QFlowLayout.hpp"

#include <unordered_map>
#include <vector>

class QDeviceStatus : public QWidget
{
    Q_OBJECT

    static constexpr uint8_t ICON_DIMENSION = 55U;
    static constexpr uint8_t DEVICE_INFO_HEIGHT = 100U;
    static constexpr uint8_t DEVICE_INFO_WIDTH = 210U;

    struct sCanDeviceInfos
    {
        uint16_t deviceMessageCount;
        uint16_t numberOfDeviceReboots;
        QWidget* deviceInfoContainer;
        QLabel* deviceInfoLabel;
    };

  public:
    QDeviceStatus(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);
    void onDashboardPage();
    void onDeviceStatusPage();

  private:
    void initializeDeviceWidget();
    void addDeviceWidget(RoverCan2::Constant::eDeviceId deviceId);
    void callbackDeviceInfos(const rover_msgs::msg::CanDeviceStatus& msg_);
    void updateDeviceInfo();
    void updateDeviceColor(RoverCan2::Constant::eDeviceId deviceID_, const rover_msgs::msg::CanDeviceStatus& msg_);
    void setStatusReport(RoverCan2::Constant::eDeviceId id_);
    void updateRebootCounter(RoverCan2::Constant::eDeviceId deviceID_);
    void resetWidget();

    void hideControls();
    void showControls();
    void hideInfos();
    void showInfos();

    std::string getDeviceName(RoverCan2::Constant::eDeviceId deviceID_) const;
    std::string getDeviceIcon(RoverCan2::Constant::eDeviceId deviceID_) const;

  private slots:
    void onRequestDeviceStatusSuccessful(bool success_, const std::string& response_);

  private:
    std::shared_ptr<rclcpp::Node> _node;
    Ui::DeviceStatus _ui;

    rclcpp::Subscription<rover_msgs::msg::CanDeviceStatus>::SharedPtr _sub_deviceStatus;
    rclcpp::Client<rover_msgs::srv::Empty>::SharedPtr _client_requestErrorStatus;

    std::unordered_map<RoverCan2::Constant::eDeviceId, sCanDeviceInfos> _canDevices;
    std::vector<QWidget*> _spacerWidgets;

    QStatusWorker _QStatusWorker;
    std::unique_ptr<QFlowLayout> _layout;
};

#endif  // QDEVICESTATUS_QDEVICESTATUS_HPP
