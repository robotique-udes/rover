#ifndef CAN_MASTER_NODE_HPP
#define CAN_MASTER_NODE_HPP

#include "CanMaster/Devices/PropulsionMotors.hpp"
#include "SharedMsg.hpp"

#include "CanMaster/Devices/Camera.hpp"
#include "rover_can2/constant.hpp"

#include <rover_can2/rover_can2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rover_msgs/msg/can_device_status.hpp>
#include <rover_msgs/msg/detail/propulsion_motor__struct.hpp>
#include <rover_msgs/srv/empty.hpp>

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

/**
 * @brief TODO when adding new device:
 * 1. Create device as member obejct.
 * 2. Add the device type to _canManager template list
 * 3. Add reference to device object to _canManager constructor arguments
 * 4. Add pointer to device object to _deviceArray
 */

class CanMasterNode : public rclcpp::Node
{
    static constexpr const char* CAN_DEVICE_STATUS_TOPIC = "/rover/can/devices_status";
    static constexpr const char* ERROR_STATE_SRV_NAME = "/rover/can/request_error_state";

  public:
    CanMasterNode();

  private:
    void CB_updateCan(void);
    void CB_ROS_canDeviceErrorStateRequest(rover_msgs::srv::Empty::Request::SharedPtr,
                                           rover_msgs::srv::Empty::Response::SharedPtr response_);
    void CB_CAN_errorStateRecv(RoverCan2::Constant::eDeviceId deviceId_, const RoverCan2::Msgs::ErrorState& canMsg_);

    bool _nodeAttachedToDevices = false;

    rclcpp::TimerBase::SharedPtr _timerUpdateCan;
    rclcpp::Publisher<rover_msgs::msg::CanDeviceStatus>::SharedPtr _pub_CanDeviceErrorState;
    rclcpp::Service<rover_msgs::srv::Empty>::SharedPtr _srv_canDeviceErrorStateRequest;

    // Shared ROS Messages
    std::shared_ptr<CanMaster::SharedRosMsg<rover_msgs::msg::PropulsionMotor>> _propMotorMsg
        = std::make_shared<CanMaster::SharedRosMsg<rover_msgs::msg::PropulsionMotor>>();

    // CanDevices
    PropulsionMotor motorFL = PropulsionMotor(RoverCan2::Constant::eDeviceId::FRONTLEFT_MOTOR,
                                                rover_msgs::msg::PropulsionMotor::FRONT_LEFT,
                                                _propMotorMsg);
    PropulsionMotor motorFR = PropulsionMotor(RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR,
                                                rover_msgs::msg::PropulsionMotor::FRONT_RIGHT,
                                                _propMotorMsg);

    PropulsionMotor motorRL = PropulsionMotor(RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR,
                                                rover_msgs::msg::PropulsionMotor::REAR_LEFT,
                                                _propMotorMsg);
    PropulsionMotor motorRR = PropulsionMotor(RoverCan2::Constant::eDeviceId::REARRIGHT_MOTOR,
                                                rover_msgs::msg::PropulsionMotor::REAR_RIGHT,
                                                _propMotorMsg);

    Camera cameraMain = Camera(RoverCan2::Constant::eDeviceId::CAMERA_ROVER_MAIN, rover_msgs::msg::CameraControl::ID_CAM_MAIN);
    Camera cameraAntenna
        = Camera(RoverCan2::Constant::eDeviceId::CAMERA_ROVER_ANTENNA, rover_msgs::msg::CameraControl::ID_CAM_ANTENNA);
    Camera cameraSideFront
        = Camera(RoverCan2::Constant::eDeviceId::CAMERA_ROVER_FRONT_SIDE, rover_msgs::msg::CameraControl::ID_CAM_FRONT_SIDE);
    Camera cameraArmTop = Camera(RoverCan2::Constant::eDeviceId::CAMERA_ARM_TOP, rover_msgs::msg::CameraControl::ID_CAM_ARM_TOP);
    Camera cameraArmSide
        = Camera(RoverCan2::Constant::eDeviceId::CAMERA_ARM_SIDE, rover_msgs::msg::CameraControl::ID_CAM_ARM_SIDE);

    // Can
    RoverCan2::Drivers::DriverMock __canDriver;
    RoverCan2::ManagerMaster<RoverCan2::Drivers::DriverMock,
                             PropulsionMotor&,
                             PropulsionMotor&,
                             PropulsionMotor&,
                             PropulsionMotor&,
                             Camera&,
                             Camera&,
                             Camera&,
                             Camera&,
                             Camera&>
        _canManager = RoverCan2::ManagerMaster(
            __canDriver,
            [this](RoverCan2::Constant::eDeviceId deviceId_, const RoverCan2::Msgs::ErrorState& msg_)
            {
                this->CB_CAN_errorStateRecv(deviceId_, msg_);
            },
            motorFL,
            motorFR,
            motorRL,
            motorRR,
            cameraMain,
            cameraAntenna,
            cameraSideFront,
            cameraArmTop,
            cameraArmSide);

    std::array<MasterDevice*, 9U> _deviceArray
        = {&motorFL, &motorFR, &motorRL, &motorRR, &cameraMain, &cameraAntenna, &cameraSideFront, &cameraArmTop, &cameraArmSide};
};

#endif  // CAN_MASTER_NODE_HPP
