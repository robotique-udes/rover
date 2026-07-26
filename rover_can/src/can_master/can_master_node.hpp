#ifndef CAN_MASTER_NODE_HPP
#define CAN_MASTER_NODE_HPP

#include "can_master/devices/arm_joint.hpp"
#include "can_master/devices/camera.hpp"
#include "can_master/devices/propulsion_motors.hpp"
#include "can_master/devices/light.hpp"
#include "can_master/devices/gnss.hpp"
#include "can_master/devices/sensor_box.hpp"
#include "can_master/devices/morse_input.hpp"
#include "rover_can2/drivers/driver_linux.hpp"

#include <rover_msgs/msg/can_device_status.hpp>
#include <rover_msgs/msg/arm_msg.hpp>
#include <rover_msgs/srv/empty.hpp>

#include <rover_can2/rover_can2.hpp>
#include <rclcpp/rclcpp.hpp>

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

/**
 * @brief When adding new devices:
 * 1. Create device as member object
 * 2. Add the device type to _canManager template list
 * 3. Add device object reference to _canManager constructor arguments
 * 4. Add device object pointer to _deviceArray
 */

class CanMasterNode : public rclcpp::Node
{
    static constexpr const char* TOPIC_NAME_CAN_DEVICE_STATUS = "/rover/can/devices_status";
    static constexpr const char* SERVICE_NAME_ERROR_STATE = "/rover/can/request_error_state";
    static constexpr const uint32_t CAN_DRIVER_UPDATE_PERIOD_MS = 1U;

  public:
    CanMasterNode();

  private:
    void CB_updateCan(void);
    void CB_ROS_canDeviceErrorStateRequest(rover_msgs::srv::Empty::Request::SharedPtr,
                                           rover_msgs::srv::Empty::Response::SharedPtr response_);
    void CB_CAN_errorStateRecv(RoverCan2::Constant::eDeviceId deviceId_, const RoverCan2::Msgs::ErrorState& canMsg_);

    bool _nodeAttachedToDevices = false;

    rclcpp::TimerBase::SharedPtr _timerUpdateCan;
    rclcpp::Publisher<rover_msgs::msg::CanDeviceStatus>::SharedPtr _pub_canDeviceErrorState;
    rclcpp::Service<rover_msgs::srv::Empty>::SharedPtr _srv_canDeviceErrorStateRequest;

    // Shared ROS Messages
    std::shared_ptr<CanMaster::SharedRosMsg<rover_msgs::msg::PropulsionMotor>> _propMotorMsg
        = std::make_shared<CanMaster::SharedRosMsg<rover_msgs::msg::PropulsionMotor>>();

    std::shared_ptr<CanMaster::SharedRosMsg<rover_msgs::msg::ArmMsg>> _armJointMsg
        = std::make_shared<CanMaster::SharedRosMsg<rover_msgs::msg::ArmMsg>>();

    // CanDevices
    PropulsionMotor motorFL = PropulsionMotor(RoverCan2::Constant::eDeviceId::FRONTLEFT_MOTOR,
                                              rover_msgs::msg::PropulsionMotor::MOTOR_FRONT_LEFT,
                                              _propMotorMsg);
    PropulsionMotor motorFR = PropulsionMotor(RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR,
                                              rover_msgs::msg::PropulsionMotor::MOTOR_FRONT_RIGHT,
                                              _propMotorMsg);
    PropulsionMotor motorRL = PropulsionMotor(RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR,
                                              rover_msgs::msg::PropulsionMotor::MOTOR_REAR_LEFT,
                                              _propMotorMsg);
    PropulsionMotor motorRR = PropulsionMotor(RoverCan2::Constant::eDeviceId::REARRIGHT_MOTOR,
                                              rover_msgs::msg::PropulsionMotor::MOTOR_REAR_RIGHT,
                                              _propMotorMsg);

    Camera cameraMain = Camera(RoverCan2::Constant::eDeviceId::CAMERA_ROVER_MAIN, rover_msgs::msg::CameraControl::ID_CAM_MAIN);
    Camera cameraAntenna
        = Camera(RoverCan2::Constant::eDeviceId::CAMERA_ROVER_ANTENNA, rover_msgs::msg::CameraControl::ID_CAM_ANTENNA);
    Camera cameraSideFront
        = Camera(RoverCan2::Constant::eDeviceId::CAMERA_ROVER_FRONT_SIDE, rover_msgs::msg::CameraControl::ID_CAM_FRONT_SIDE);
    Camera cameraArmTop = Camera(RoverCan2::Constant::eDeviceId::CAMERA_ARM_TOP, rover_msgs::msg::CameraControl::ID_CAM_ARM_TOP);
    Camera cameraArmSide
        = Camera(RoverCan2::Constant::eDeviceId::CAMERA_ARM_SIDE, rover_msgs::msg::CameraControl::ID_CAM_ARM_SIDE);

    Gnss gnss = Gnss(RoverCan2::Constant::eDeviceId::GNSS);

    ArmJoint JL = ArmJoint(RoverCan2::Constant::eDeviceId::JL_CONTROLLER, rover_msgs::msg::ArmMsg::JL, _armJointMsg);
    // ArmJoint J0 = ArmJoint(RoverCan2::Constant::eDeviceId::JR_CONTROLLER, rover_msgs::msg::ArmMsg::J0, _armJointMsg);
    ArmJoint J1 = ArmJoint(RoverCan2::Constant::eDeviceId::J1_CONTROLLER, rover_msgs::msg::ArmMsg::J1, _armJointMsg);
    ArmJoint J2 = ArmJoint(RoverCan2::Constant::eDeviceId::J2_CONTROLLER, rover_msgs::msg::ArmMsg::J2, _armJointMsg);
    ArmJoint gripperTilt
        = ArmJoint(RoverCan2::Constant::eDeviceId::GRIPPER_TILT_CONTROLLER, rover_msgs::msg::ArmMsg::GRIPPER_TILT, _armJointMsg);
    ArmJoint gripperRot
        = ArmJoint(RoverCan2::Constant::eDeviceId::GRIPPER_ROT_CONTROLLER, rover_msgs::msg::ArmMsg::GRIPPER_ROT, _armJointMsg);
    ArmJoint gripperClose = ArmJoint(RoverCan2::Constant::eDeviceId::GRIPPER_CLOSE_CONTROLLER,
                                     rover_msgs::msg::ArmMsg::GRIPPER_CLOSE,
                                     _armJointMsg);

    MorseInput morseInput = MorseInput(RoverCan2::Constant::eDeviceId::MORSE_CODE);

    Light lightMain = Light(RoverCan2::Constant::eDeviceId::LIGHTS_MAIN);

    SensorBox sensorBox;

    // Can
    RoverCan2::Drivers::DriverLinux __canDriver;
    RoverCan2::ManagerMaster<RoverCan2::Drivers::DriverLinux,
                             PropulsionMotor&,
                             PropulsionMotor&,
                             PropulsionMotor&,
                             PropulsionMotor&,
                             Camera&,
                             Camera&,
                             Camera&,
                             Camera&,
                             Camera&,
                             Gnss&,
                             ArmJoint&,
                             ArmJoint&,
                             ArmJoint&,
                             // ArmJoint&,
                             ArmJoint&,
                             ArmJoint&,
                             ArmJoint&,
                             Light&,
                             SensorBox&,
                             MorseInput&>
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
            cameraArmSide,
            gnss,
            JL,
            // J0,
            J1,
            J2,
            gripperTilt,
            gripperRot,
            gripperClose,
            lightMain,
            sensorBox,
            morseInput);

    std::array<MasterDevice*, 19U> _deviceArray = {&motorFL,
                                                   &motorFR,
                                                   &motorRL,
                                                   &motorRR,
                                                   &cameraMain,
                                                   &cameraAntenna,
                                                   &cameraSideFront,
                                                   &cameraArmTop,
                                                   &cameraArmSide,
                                                   &gnss,
                                                   &JL,
                                                   //&J0,
                                                   &J1,
                                                   &J2,
                                                   &gripperTilt,
                                                   &gripperRot,
                                                   &gripperClose,
                                                   &lightMain,
                                                   &sensorBox,
                                                   &morseInput};
};

#endif  // CAN_MASTER_NODE_HPP
