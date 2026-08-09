#ifndef __QARBITRATION_HPP__
#define __QARBITRATION_HPP__

#include "ui_Arbitration.h"

#include "rover_lib2/helpers/log.hpp"

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/joy_demux_status.hpp>
#include <rover_msgs/msg/drivetrain_arbitration.hpp>
#include <rover_msgs/srv/joy_demux_set_state.hpp>
#include <rover_msgs/srv/drive_train_arbitration.hpp>

// QT
#include <QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>

#include <string>

DEFINE_LOG_NODE(QArbitration, Logger::eNodeState::OFF);

class QArbitration : public QWidget
{
    Q_OBJECT

    static constexpr const char* TOPIC_JOY_DEMUX_CONTROL = "/base/joy/demux_control";
    static constexpr const char* TOPIC_DT_DEMUX_CONTROL = "/rover/drive_train/demux_control";
    static constexpr const char* TOPIC_JOY_DEMUX_STATUS = "/base/joy/demux_status";
    static constexpr const char* TOPIC_DT_DEMUX_STATUS = "/rover/drive_train/demux_status";

    enum class eControllerType : uint8_t
    {
        MAIN = rover_msgs::srv::JoyDemuxSetState_Request::CONTROLLER_MAIN,
        SECONDARY = rover_msgs::srv::JoyDemuxSetState_Request::CONTROLLER_SECONDARY
    };

    enum class eDemuxDestination : uint8_t
    {
        DRIVE_TRAIN = rover_msgs::srv::JoyDemuxSetState_Request::DEST_DRIVE_TRAIN,
        ARM = rover_msgs::srv::JoyDemuxSetState_Request::DEST_ARM,
        ANTENNA = rover_msgs::srv::JoyDemuxSetState_Request::DEST_ANTENNA,
        SCIENCE = rover_msgs::srv::JoyDemuxSetState_Request::DEST_SCIENCE,
        NONE = rover_msgs::srv::JoyDemuxSetState_Request::DEST_NONE
    };

    enum class eDriveTrainDestination : uint8_t
    {
        NONE = rover_msgs::msg::DrivetrainArbitration::NONE,
        TELEOP = rover_msgs::msg::DrivetrainArbitration::TELEOP,
        AUTONOMUS = rover_msgs::msg::DrivetrainArbitration::AUTONOMUS
    };

  public:
    QArbitration(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);

  signals:
    void joyDemuxStatusChanged(const rover_msgs::msg::JoyDemuxStatus::SharedPtr msg_);
    void driveTrainDemuxStatusChanged(const rover_msgs::msg::DrivetrainArbitration::SharedPtr msg_);

  private slots:
    void onControllerComboChanged(eControllerType controller_, int index_);
    void onDriveTrainComboChanged(int index_);
    void onJoyDemuxStatusChanged(const rover_msgs::msg::JoyDemuxStatus::SharedPtr msg_);
    void onDriveTrainDemuxStatusChanged(const rover_msgs::msg::DrivetrainArbitration::SharedPtr msg_);

  private:
    void initComboBoxItems();

    template<typename T>
    void checkServiceAvailable(rclcpp::Client<T>::SharedPtr client_, const std::string& serviceName_);

    std::shared_ptr<rclcpp::Node> _node;
    Ui::Arbitration _ui;

    rclcpp::Subscription<rover_msgs::msg::JoyDemuxStatus>::SharedPtr _joyDemuxStatusSub;
    rclcpp::Subscription<rover_msgs::msg::DrivetrainArbitration>::SharedPtr _driveTrainStatusSub;
    rclcpp::Client<rover_msgs::srv::JoyDemuxSetState>::SharedPtr _clientJoy;
    rclcpp::Client<rover_msgs::srv::DriveTrainArbitration>::SharedPtr _clientDriveTrain;
};

#endif
