#ifndef __QARBITRATION_HPP__
#define __QARBITRATION_HPP__
#include "UI_Arbitration.h"
// ROS
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/srv/joy_demux_set_state.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"
#include "rover_msgs/srv/light_control.hpp"
#include "rovus_lib/macros.h"
// QT
#include <QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <QMessageBox>

#include <QTimer>


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
    QArbitration(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_): QWidget(parent_), _guiNode(guiNode_)
    {
        _ui.setupUi(this);

        _sub_joyDemux = _guiNode->create_subscription<rover_msgs::msg::JoyDemuxStatus>(
				"/joy/demux/status",
				1,
				std::bind(&QArbitration::joyDemuxCallback, this, std::placeholders::_1));

        QButtonGroup* mainButtonGroup = new QButtonGroup(this);
        mainButtonGroup->addButton(_ui.checkBox);
        mainButtonGroup->addButton(_ui.checkBox_3);
        mainButtonGroup->addButton(_ui.checkBox_4);
        mainButtonGroup->addButton(_ui.checkBox_2);
        mainButtonGroup->setExclusive(true);

        QButtonGroup* secondaryButtonGroup = new QButtonGroup(this);
        secondaryButtonGroup->addButton(_ui.checkBox_5);
        secondaryButtonGroup->addButton(_ui.checkBox_7);
        secondaryButtonGroup->addButton(_ui.checkBox_6);
        secondaryButtonGroup->addButton(_ui.checkBox_8);
        secondaryButtonGroup->setExclusive(true);

        //MAIN
        connect(_ui.checkBox, &QCheckBox::stateChanged, this, [this](int8_t state){
          if (state == Qt::Checked){
            sendRequest(main, drive_train);
          }
        });
        connect(_ui.checkBox_3, &QCheckBox::stateChanged, this, [this](int8_t state){
          if (state == Qt::Checked){
            sendRequest(main, arm);
          }
        });
        connect(_ui.checkBox_4, &QCheckBox::stateChanged, this, [this](int8_t state){
          if (state == Qt::Checked){
            sendRequest(main, antenna);
          }
        });
        connect(_ui.checkBox_2, &QCheckBox::stateChanged, this, [this](int8_t state){
          if (state == Qt::Checked){
            sendRequest(main, none);
          }
        });

        //SECONDARY
          connect(_ui.checkBox_5, &QCheckBox::stateChanged, this, [this](int8_t state){
          if (state == Qt::Checked){
            sendRequest(secondary, drive_train);
          }
        });
        connect(_ui.checkBox_7, &QCheckBox::stateChanged, this, [this](int8_t state){
          if (state == Qt::Checked){
            sendRequest(secondary, arm);
          }
        });
        connect(_ui.checkBox_6, &QCheckBox::stateChanged, this, [this](int8_t state){
          if (state == Qt::Checked){
            sendRequest(secondary, antenna);
          }
        });
        connect(_ui.checkBox_8, &QCheckBox::stateChanged, this, [this](int8_t state){
          if (state == Qt::Checked){
            sendRequest(secondary, none);
          }
        });

        _clientJoy = _guiNode->create_client<rover_msgs::srv::JoyDemuxSetState>("/demux_control");
        _clientLights = _guiNode->create_client<rover_msgs::srv::LightControl>("/rover/auxiliary/set/lights");
        
        _ui.checkBox_2->setChecked(true);  
        _ui.checkBox_8->setChecked(true);  

        
    }

    ~QArbitration() {};

  private:
  	void joyDemuxCallback(const rover_msgs::msg::JoyDemuxStatus::SharedPtr rosMsg_);
    void setControlButtonsEnabled(bool enabled);

		
		rclcpp::Subscription<rover_msgs::msg::JoyDemuxStatus>::SharedPtr _sub_joyDemux;
    Ui::Form _ui;
    std::shared_ptr<rclcpp::Node> _guiNode;
    rclcpp::Client<rover_msgs::srv::JoyDemuxSetState>::SharedPtr _clientJoy;
    rclcpp::Client<rover_msgs::srv::LightControl>::SharedPtr _clientLights;
    QTimer* _reconnectTimer = nullptr;


  private slots:
    void sendRequest(eControllerType controllerType_, eDemuxDestination demuxDestination_);
};
#endif