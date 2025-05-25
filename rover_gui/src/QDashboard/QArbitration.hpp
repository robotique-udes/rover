#ifndef __QARBITRATION_HPP__
#define __QARBITRATION_HPP__

#include "UI_Arbitration.h"
#include "rover_lib2/helpers/log.hpp"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"
#include "rover_msgs/msg/drivetrain_arbitration.hpp"
#include "rover_msgs/srv/joy_demux_set_state.hpp"
#include "rover_msgs/srv/drive_train_arbitration.hpp"

// QT
#include <QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <rover_msgs/msg/detail/drivetrain_arbitration__struct.hpp>
#include <string>

DEFINE_LOG_NODE(QArbitration, Logger::eNodeState::OFF);

class QArbitration : public QWidget
{
    Q_OBJECT

    static constexpr const char* TOPIC_JOY_DEMUX_CONTROL = "/base/joy/demux_control";
    static constexpr const char* TOPIC_DT_DEMUX_CONTROL = "/rover/drive_train/demux_control";
    static constexpr const char* TOPIC_JOY_DEMUX_STATUS  = "/base/joy/demux_status";
    static constexpr const char* TOPIC_DT_DEMUX_STATUS = "/rover/drive_train/demux_status";


    enum class eControllerType
    {
        main = (int8_t)rover_msgs::srv::JoyDemuxSetState_Request::CONTROLLER_MAIN,
        secondary = (int8_t)rover_msgs::srv::JoyDemuxSetState_Request::CONTROLLER_SECONDARY
    };

    enum class eDemuxDestination
    {
        drive_train = (int8_t)rover_msgs::srv::JoyDemuxSetState_Request::DEST_DRIVE_TRAIN,
        arm = (int8_t)rover_msgs::srv::JoyDemuxSetState_Request::DEST_ARM,
        antenna = (int8_t)rover_msgs::srv::JoyDemuxSetState_Request::DEST_ANTENNA,
        none = (int8_t)rover_msgs::srv::JoyDemuxSetState_Request::DEST_NONE
    };

    enum class eDriveTrainDestination
    {
        none = (int8_t)rover_msgs::msg::DrivetrainArbitration::NONE,
        teleop = (int8_t)rover_msgs::msg::DrivetrainArbitration::TELEOP,
        autonomus = (int8_t)rover_msgs::msg::DrivetrainArbitration::AUTONOMUS
    };

  public:
    QArbitration(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);
    ~QArbitration(){};

  private slots:
    void onMainComboChanged(int index);
    void onSecComboChanged(int index);
    void onDriveTrainComboChanged(int index);

  private:
    void initComboBoxItems();
    void JoyDemuxStatusCallback(const rover_msgs::msg::JoyDemuxStatus::SharedPtr msg);
    void DriveTrainDemuxStatusCallback(const rover_msgs::msg::DrivetrainArbitration::SharedPtr msg);
    
    template<typename T>
    void checkServiceAvailable(rclcpp::Client<T>::SharedPtr client, const std::string& serviceName);

    std::shared_ptr<rclcpp::Node> _node;
    Ui::Arbitration _ui;

    rclcpp::Subscription<rover_msgs::msg::JoyDemuxStatus>::SharedPtr _joyDemuxStatusSub;
    rclcpp::Subscription<rover_msgs::msg::DrivetrainArbitration>::SharedPtr _driveTrainStatusSub;
    rclcpp::Client<rover_msgs::srv::JoyDemuxSetState>::SharedPtr _clientJoy;
    rclcpp::Client<rover_msgs::srv::DriveTrainArbitration>::SharedPtr _clientDriveTrain;
};

#endif
