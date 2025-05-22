#ifndef __QARBITRATION_HPP__
#define __QARBITRATION_HPP__
#include "UI_Arbitration.h"
// ROS
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"

// QT
#include <QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <QMessageBox>

#include <QTimer>

class QArbitration : public QWidget
{
    Q_OBJECT

    /*
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
    */

  public:
    QArbitration(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);

    ~QArbitration() {};

  private:
    void initComboBoxItems();

  std::shared_ptr<rclcpp::Node> _node;
  Ui::Arbitration _ui;
  rclcpp::Subscription<rover_msgs::msg::JoyDemuxStatus>::SharedPtr _demuxStatusSub;
};
#endif