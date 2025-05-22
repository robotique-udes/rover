#ifndef __QARBITRATION_HPP__
#define __QARBITRATION_HPP__

#include "UI_Arbitration.h"
#include "rover_lib2/helpers/log.hpp"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"
#include "rover_msgs/srv/joy_demux_set_state.hpp"

// QT
#include <QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>

DEFINE_LOG_NODE(QArbitration, Logger::eNodeState::OFF);

class QArbitration : public QWidget
{
    Q_OBJECT

    
    enum eControllerType
    {
        main = (int8_t)rover_msgs::srv::JoyDemuxSetState_Request::CONTROLLER_MAIN,
        secondary = (int8_t)rover_msgs::srv::JoyDemuxSetState_Request::CONTROLLER_SECONDARY
    };

    enum eDemuxDestination
    {
        drive_train = (int8_t)rover_msgs::srv::JoyDemuxSetState_Request::DEST_DRIVE_TRAIN,
        arm = (int8_t)rover_msgs::srv::JoyDemuxSetState_Request::DEST_ARM,
        antenna = (int8_t)rover_msgs::srv::JoyDemuxSetState_Request::DEST_ANTENNA,
        none = (int8_t)rover_msgs::srv::JoyDemuxSetState_Request::DEST_NONE
    };
    

  public:
    QArbitration(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);

    ~QArbitration() {};

  private slots:
    void onMainComboChanged(int index);
    void onSecComboChanged(int index);

  private:
    void initComboBoxItems();

    std::shared_ptr<rclcpp::Node> _node;
    Ui::Arbitration _ui;

    rclcpp::Subscription<rover_msgs::msg::JoyDemuxStatus>::SharedPtr _demuxStatusSub;
    rclcpp::Client<rover_msgs::srv::JoyDemuxSetState>::SharedPtr _clientJoy;
};
#endif