// Network socket stuff
#include <fcntl.h>
#include <net/if.h>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/arm_msg.hpp"
#include "rover_msgs/msg/camera_angle.hpp"
#include "rover_msgs/msg/camera_control.hpp"
#include "rover_msgs/msg/can_device_status.hpp"
#include "rover_msgs/msg/compass.hpp"
#include "rover_msgs/msg/gps.hpp"
#include "rover_msgs/msg/light_control.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include <chrono>

// RoverCanLib
#include "rover_can_lib/can_device.hpp"
#include "rover_can_lib/config.hpp"
#include "rover_can_lib/msgs/arm_cmd.hpp"
#include "rover_can_lib/msgs/arm_status.hpp"
#include "rover_can_lib/msgs/cam_control.hpp"
#include "rover_can_lib/msgs/cam_control_a2.hpp"
#include "rover_can_lib/msgs/cam_pan.hpp"
#include "rover_can_lib/msgs/compass.hpp"
#include "rover_can_lib/msgs/gps.hpp"
#include "rover_can_lib/msgs/heartbeat.hpp"
#include "rover_can_lib/msgs/light_control.hpp"
#include "rover_can_lib/msgs/propulsion_motor_cmd.hpp"
#include "rover_can_lib/msgs/propulsion_motor_status.hpp"

#define LOGGER_NAME "CanMasterNode"

// Forward declarations
int createSocket(const char* canNetworkName_);
void signal_handler(int signo_);

// =============================================================================
// Global objects
volatile sig_atomic_t g_shutdownFlag = 0;

// =============================================================================
//  Per devices msg global object
// PDB
RoverCanLib::Msgs::camControl msg_CAN_CameraA2;
RoverCanLib::Msgs::camControl msg_CAN_CameraR1M_1;
RoverCanLib::Msgs::camControl msg_CAN_CameraR1M_2;
RoverCanLib::Msgs::camControl msg_CAN_CameraR1M_3;

// Motors
RoverCanLib::Msgs::PropulsionMotorStatus msg_CAN_PropFrontLeft;
RoverCanLib::Msgs::PropulsionMotorStatus msg_CAN_PropFrontRight;
RoverCanLib::Msgs::PropulsionMotorStatus msg_CAN_PropRearLeft;
RoverCanLib::Msgs::PropulsionMotorStatus msg_CAN_PropRearRight;

// Arm
RoverCanLib::Msgs::armStatus msg_CAN_ArmJl;
RoverCanLib::Msgs::armStatus msg_CAN_ArmJ0;
RoverCanLib::Msgs::armStatus msg_CAN_ArmJ1;
RoverCanLib::Msgs::armStatus msg_CAN_ArmJ2;
RoverCanLib::Msgs::armStatus msg_CAN_ArmGripperTilt;
RoverCanLib::Msgs::armStatus msg_CAN_ArmGripperRot;
RoverCanLib::Msgs::armStatus msg_CAN_ArmGripperClose;

// Aux
RoverCanLib::Msgs::GPS msg_CAN_gps;
RoverCanLib::Msgs::Compass msg_CAN_compass;
// =============================================================================

// =============================================================================
//  Global ROS msg object
rover_msgs::msg::PropulsionMotor msg_ROS_propMotor;
rover_msgs::msg::ArmMsg msg_ROS_arm;
rover_msgs::msg::Gps msg_ROS_gps;
rover_msgs::msg::Compass msg_ROS_compass;
// =============================================================================

class CanMaster : public rclcpp::Node
{
  public:
    CanMaster(int canSocket_);

  private:
    void mainLoop();
    // Sends heartbeat on the can network at specific rate
    RoverCanLib::Constant::eInternalErrorCode updateHeartbeat();
    // Read all msg in the rx queue and parse them accordingly. Also resets watchdog if a watchdog msg is received
    RoverCanLib::Constant::eInternalErrorCode readMsgFromCanSocket();
    // Send a ErrorState msg to every devices registered in deviceMap.
    // They should all answer with their receptive state
    RoverCanLib::Constant::eInternalErrorCode askStateCanDevices();

    int _canSocket;
    RoverLib::Timer<uint64_t, RoverLib::millis> _timerHeartbeat
        = RoverLib::Timer<uint64_t, RoverLib::millis>((uint64_t)(1000.0f / RoverCanLib::Constant::HEARTBEAT_FREQ));
    RoverLib::Chrono<uint64_t, RoverLib::millis> _chonoCanWatchdog;
    rclcpp::TimerBase::SharedPtr _timerLoop;
    std::unordered_map<size_t, CanDevice> _deviceMap;
    std::unordered_map<size_t, RoverCanLib::Msgs::Msg*> _msgsMap;

    // =========================================================================
    //  Device loop timers
    RoverLib::Timer<unsigned long, RoverLib::millis> _timer_motorCmdSend = RoverLib::Timer<unsigned long, RoverLib::millis>(50);
    RoverLib::Timer<unsigned long, RoverLib::millis> _timer_armCmdSend = RoverLib::Timer<unsigned long, RoverLib::millis>(50);
    // =========================================================================

    // =========================================================================
    //  Device publishers
    rclcpp::Publisher<rover_msgs::msg::CanDeviceStatus>::SharedPtr _pub_canStatus;
    rclcpp::Publisher<rover_msgs::msg::PropulsionMotor>::SharedPtr _pub_propulsionMotor;
    rclcpp::Publisher<rover_msgs::msg::ArmMsg>::SharedPtr _pub_arm;
    rclcpp::Publisher<rover_msgs::msg::Gps>::SharedPtr _pub_gps;
    rclcpp::Publisher<rover_msgs::msg::Compass>::SharedPtr _pub_compass;

    // =========================================================================

    // =========================================================================
    //  Device subscriber
    rclcpp::Subscription<rover_msgs::msg::PropulsionMotor>::SharedPtr _sub_propulsionMotor;
    rclcpp::Subscription<rover_msgs::msg::ArmMsg>::SharedPtr _sub_arm;
    rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr _sub_cameras;
    rclcpp::Subscription<rover_msgs::msg::LightControl>::SharedPtr _sub_lights;
    rclcpp::Subscription<rover_msgs::msg::CameraAngle>::SharedPtr _sub_cameraPano;
    // =========================================================================

    // =========================================================================
    //  Devices callbacks
    void CB_Can_None(uint16_t dontUse0_, const can_frame* dontUse1_);
    void CB_Can_PropulsionMotor(uint16_t id_, const can_frame* frameMsg);
    void CB_Can_Arm(uint16_t id_, const can_frame* frameMsg);
    void CB_Can_GPS(uint16_t id_, const can_frame* frameMsg);
    void CB_Can_Compass(uint16_t id_, const can_frame* frameMsg);
    // =========================================================================

    // =========================================================================
    //  Ros callbacks
    void CB_ROS_propulsionMotor(const rover_msgs::msg::PropulsionMotor::SharedPtr msg);
    void CB_ROS_Arm(const rover_msgs::msg::ArmMsg::SharedPtr msg);
    void CB_ROS_cameraControl(const rover_msgs::msg::CameraControl::SharedPtr rosMsg);
    void CB_ROS_lightControl(const rover_msgs::msg::LightControl::SharedPtr rosMsg);
    void CB_ROS_cameraPano(const rover_msgs::msg::CameraAngle::SharedPtr rosMsg);
    // =========================================================================
};

int main(int argc, char* argv[])
{
    signal(SIGINT, signal_handler);  // Makes CTRL+C work

    while (!g_shutdownFlag)
    {
        rclcpp::init(argc, argv);
        int canSocket = createSocket("canRovus");
        assert(canSocket > 0);

        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Canbus ready, starting loop");
        rclcpp::spin(std::make_shared<CanMaster>(canSocket));

        close(canSocket);
        sleep(1);
    }

    return 0;
}

// Ctrl+C signal handler
void signal_handler(int signo_)
{
    RCLCPP_INFO(rclcpp::get_logger("[SHUTDOWN]"), "Shutdown asked by user, exiting...");
    (void)signo_;
    g_shutdownFlag = 1;
}

// Returns a socket on success or < 0 on error
int createSocket(const char* canNetworkName_)
{
    int canSocket;
    if ((canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        RCLCPP_FATAL(rclcpp::get_logger(LOGGER_NAME), "Error creating socket");
        return -1;
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = if_nametoindex(canNetworkName_);
    if (bind(canSocket, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        RCLCPP_FATAL(rclcpp::get_logger(LOGGER_NAME), "Error bind to socket");
        close(canSocket);
        return -1;
    }

    // Set socket to non-blocking mode
    if (fcntl(canSocket, F_SETFL, O_NONBLOCK) < 0)
    {
        RCLCPP_FATAL(rclcpp::get_logger(LOGGER_NAME), "Error updating socket parameters");
        close(canSocket);
        return -1;
    }

    return canSocket;
}

// =============================================================================
//  Class method definitions
// =============================================================================
CanMaster::CanMaster(int canSocket_): Node("can_master")
{
    _canSocket = canSocket_;

    // =========================================================================
    //  Device publishers actual creation
    _pub_canStatus = this->create_publisher<rover_msgs::msg::CanDeviceStatus>("/rover/can/device_status", 1);
    _pub_propulsionMotor = this->create_publisher<rover_msgs::msg::PropulsionMotor>("/rover/drive_train/status/prop_motor", 1);
    _pub_arm = this->create_publisher<rover_msgs::msg::ArmMsg>("/rover/arm/status/current_positions", 1);
    _pub_gps = this->create_publisher<rover_msgs::msg::Gps>("/rover/gps/position", 1);
    _pub_compass = this->create_publisher<rover_msgs::msg::Compass>("/rover/auxiliary/compass/raw", 1);
    // =========================================================================

    // Add messages type to msgsMap
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::CAMERA_A2] = &msg_CAN_CameraA2;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::CAMERA_R1M_1] = &msg_CAN_CameraR1M_1;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::CAMERA_R1M_2] = &msg_CAN_CameraR1M_2;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::CAMERA_R1M_3] = &msg_CAN_CameraR1M_3;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::FRONTLEFT_MOTOR] = &msg_CAN_PropFrontLeft;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::FRONTRIGHT_MOTOR] = &msg_CAN_PropFrontRight;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::REARLEFT_MOTOR] = &msg_CAN_PropRearLeft;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::REARRIGHT_MOTOR] = &msg_CAN_PropRearRight;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::JL_CONTROLLER] = &msg_CAN_ArmJl;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::J0_CONTROLLER] = &msg_CAN_ArmJ0;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::J1_CONTROLLER] = &msg_CAN_ArmJ1;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::J2_CONTROLLER] = &msg_CAN_ArmJ2;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::GRIPPER_TILT_CONTROLLER] = &msg_CAN_ArmGripperTilt;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::GRIPPER_ROT_CONTROLLER] = &msg_CAN_ArmGripperRot;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::GRIPPER_CLOSE_CONTROLLER] = &msg_CAN_ArmGripperClose;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::GPS] = &msg_CAN_gps;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::COMPASS] = &msg_CAN_compass;

    // =========================================================================
    //  Devices objects constructors

    // PDB
    _deviceMap.emplace(
        (size_t)RoverCanLib::Constant::eDeviceId::PDB_CONTROLLER,
        CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::PDB_CONTROLLER, this, &CanMaster::CB_Can_None, _pub_canStatus));
    _deviceMap.emplace(
        (size_t)RoverCanLib::Constant::eDeviceId::CAMERA_A2,
        CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::CAMERA_A2, this, &CanMaster::CB_Can_None, _pub_canStatus));
    _deviceMap.emplace(
        (size_t)RoverCanLib::Constant::eDeviceId::CAMERA_R1M_1,
        CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::CAMERA_R1M_1, this, &CanMaster::CB_Can_None, _pub_canStatus));
    _deviceMap.emplace(
        (size_t)RoverCanLib::Constant::eDeviceId::CAMERA_R1M_2,
        CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::CAMERA_R1M_2, this, &CanMaster::CB_Can_None, _pub_canStatus));
    _deviceMap.emplace(
        (size_t)RoverCanLib::Constant::eDeviceId::CAMERA_R1M_3,
        CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::CAMERA_R1M_3, this, &CanMaster::CB_Can_None, _pub_canStatus));
    _deviceMap.emplace(
        (size_t)RoverCanLib::Constant::eDeviceId::LIGHTS,
        CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::LIGHTS, this, &CanMaster::CB_Can_None, _pub_canStatus));

    // Motor Drive
    _deviceMap.emplace((size_t)RoverCanLib::Constant::eDeviceId::FRONTLEFT_MOTOR,
                       CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::FRONTLEFT_MOTOR,
                                 this,
                                 &CanMaster::CB_Can_PropulsionMotor,
                                 _pub_canStatus));
    _deviceMap.emplace((size_t)RoverCanLib::Constant::eDeviceId::FRONTRIGHT_MOTOR,
                       CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::FRONTRIGHT_MOTOR,
                                 this,
                                 &CanMaster::CB_Can_PropulsionMotor,
                                 _pub_canStatus));
    _deviceMap.emplace((size_t)RoverCanLib::Constant::eDeviceId::REARLEFT_MOTOR,
                       CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::REARLEFT_MOTOR,
                                 this,
                                 &CanMaster::CB_Can_PropulsionMotor,
                                 _pub_canStatus));
    _deviceMap.emplace((size_t)RoverCanLib::Constant::eDeviceId::REARRIGHT_MOTOR,
                       CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::REARRIGHT_MOTOR,
                                 this,
                                 &CanMaster::CB_Can_PropulsionMotor,
                                 _pub_canStatus));

    // Arm
    _deviceMap.emplace(
        (size_t)RoverCanLib::Constant::eDeviceId::JL_CONTROLLER,
        CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::JL_CONTROLLER, this, &CanMaster::CB_Can_Arm, _pub_canStatus));
    _deviceMap.emplace(
        (size_t)RoverCanLib::Constant::eDeviceId::J0_CONTROLLER,
        CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::J0_CONTROLLER, this, &CanMaster::CB_Can_Arm, _pub_canStatus));
    _deviceMap.emplace(
        (size_t)RoverCanLib::Constant::eDeviceId::J1_CONTROLLER,
        CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::J1_CONTROLLER, this, &CanMaster::CB_Can_Arm, _pub_canStatus));
    _deviceMap.emplace(
        (size_t)RoverCanLib::Constant::eDeviceId::J2_CONTROLLER,
        CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::J2_CONTROLLER, this, &CanMaster::CB_Can_Arm, _pub_canStatus));
    _deviceMap.emplace((size_t)RoverCanLib::Constant::eDeviceId::GRIPPER_TILT_CONTROLLER,
                       CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::GRIPPER_TILT_CONTROLLER,
                                 this,
                                 &CanMaster::CB_Can_Arm,
                                 _pub_canStatus));
    _deviceMap.emplace((size_t)RoverCanLib::Constant::eDeviceId::GRIPPER_ROT_CONTROLLER,
                       CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::GRIPPER_ROT_CONTROLLER,
                                 this,
                                 &CanMaster::CB_Can_Arm,
                                 _pub_canStatus));
    _deviceMap.emplace((size_t)RoverCanLib::Constant::eDeviceId::GRIPPER_CLOSE_CONTROLLER,
                       CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::GRIPPER_CLOSE_CONTROLLER,
                                 this,
                                 &CanMaster::CB_Can_Arm,
                                 _pub_canStatus));

    // Aux
    _deviceMap.emplace((size_t)RoverCanLib::Constant::eDeviceId::GPS,
                       CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::GPS, this, &CanMaster::CB_Can_GPS, _pub_canStatus));
    _deviceMap.emplace(
        (size_t)RoverCanLib::Constant::eDeviceId::COMPASS,
        CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::COMPASS, this, &CanMaster::CB_Can_Compass, _pub_canStatus));
    // =========================================================================

    this->askStateCanDevices();

    // =========================================================================
    //  Topic subscriber actual creation
    _sub_propulsionMotor = this->create_subscription<rover_msgs::msg::PropulsionMotor>(
        "/rover/drive_train/cmd/out/motors",
        1,
        std::bind(&CanMaster::CB_ROS_propulsionMotor, this, std::placeholders::_1));
    _sub_arm = this->create_subscription<rover_msgs::msg::ArmMsg>("/rover/arm/cmd/goal_pos",
                                                                  1,
                                                                  std::bind(&CanMaster::CB_ROS_Arm, this, std::placeholders::_1));
    // _sub_cameras = this->create_subscription<rover_msgs::msg::CameraControl>("/TODO/CAM_TOPIC", 1,
    // std::bind(&CanMaster::CB_ROS_cameraControl, this, std::placeholders::_1));
    _sub_lights = this->create_subscription<rover_msgs::msg::LightControl>(
        "/rover/auxiliary/lights/status",
        1,
        std::bind(&CanMaster::CB_ROS_lightControl, this, std::placeholders::_1));
    _sub_cameraPano = this->create_subscription<rover_msgs::msg::CameraAngle>(
        "/rover/auxiliary/camera_pano",
        1,
        std::bind(&CanMaster::CB_ROS_cameraPano, this, std::placeholders::_1));
    // =========================================================================

    // Created last to make sure everything is init before it gets called
    _timerLoop = this->create_wall_timer(std::chrono::microseconds(10), std::bind(&CanMaster::mainLoop, this));
}

void CanMaster::mainLoop()
{
    // Send heartbeat at 4 Hz (see define)
    if (this->updateHeartbeat() != RoverCanLib::Constant::eInternalErrorCode::OK)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Error while sending Heartbeat, trying to reconnect to the can network...");
        rclcpp::shutdown();
    }

    // Checking watchdog
    if (_chonoCanWatchdog.getTime() > RoverCanLib::Constant::WATCHDOG_TIMEOUT_MS)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Watchdog triggered, trying to reconnect to the can network...");
        rclcpp::shutdown();
    }

    // Empty rx queue
    if (readMsgFromCanSocket() != RoverCanLib::Constant::eInternalErrorCode::OK)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Error detected on CAN socket, trying to reconnect to the network...");
        rclcpp::shutdown();
    }
}

RoverCanLib::Constant::eInternalErrorCode CanMaster::updateHeartbeat()
{
    if (_timerHeartbeat.isDone())
    {
        RoverCanLib::Msgs::Heartbeat msgHeartbeat;

        can_frame canFrame;
        canFrame.can_id = (canid_t)RoverCanLib::Constant::eDeviceId::MASTER_COMPUTER_UNIT;
        msgHeartbeat.getMsg((uint8_t)RoverCanLib::Msgs::Heartbeat::eMsgID::DONT_USE, &canFrame, rclcpp::get_logger(LOGGER_NAME));

        if (write(_canSocket, &canFrame, sizeof(canFrame)) != sizeof(canFrame))
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Error while writting");
            return RoverCanLib::Constant::eInternalErrorCode::ERROR;
        }
    }

    return RoverCanLib::Constant::eInternalErrorCode::OK;
}

RoverCanLib::Constant::eInternalErrorCode CanMaster::readMsgFromCanSocket()
{
    for (EVER)
    {
        struct can_frame frame;
        ssize_t bytes_read;
        bytes_read = read(_canSocket, &frame, sizeof(struct can_frame));
        if (bytes_read == sizeof(can_frame))
        {
            _chonoCanWatchdog.restart();

            // Check if the device exist in the hash map and parse can msg
            if (_deviceMap.find((size_t)frame.can_id) != _deviceMap.end())
            {
                _deviceMap.find((size_t)frame.can_id)->second.parseMsg(&frame);
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                            "Received msg with device ID: 0x%.3x which isn't defined in deviceMap, dropping",
                            frame.can_id);
            }
            continue;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return RoverCanLib::Constant::eInternalErrorCode::OK;
            break;
        }
        if (bytes_read == -1 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
            return RoverCanLib::Constant::eInternalErrorCode::ERROR;
            break;
        }
    }

    return RoverCanLib::Constant::eInternalErrorCode::OK;
}

RoverCanLib::Constant::eInternalErrorCode CanMaster::askStateCanDevices()
{
    RoverCanLib::Msgs::ErrorState msgErrorState;
    msgErrorState.data.error = 0;
    msgErrorState.data.warning = 0;

    return msgErrorState.sendMsg(RoverCanLib::Constant::eDeviceId::MASTER_COMPUTER_UNIT,
                                 _canSocket,
                                 rclcpp::get_logger(LOGGER_NAME));
}

// =============================================================================
//  Devices callbacks
void CanMaster::CB_Can_PropulsionMotor(uint16_t id_, const can_frame* frameMsg_)
{
    if (id_ != (size_t)RoverCanLib::Constant::eDeviceId::FRONTLEFT_MOTOR
        && id_ != (size_t)RoverCanLib::Constant::eDeviceId::FRONTRIGHT_MOTOR
        && id_ != (size_t)RoverCanLib::Constant::eDeviceId::REARLEFT_MOTOR
        && id_ != (size_t)RoverCanLib::Constant::eDeviceId::REARRIGHT_MOTOR)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Provided device ID: 0x%.3x isn't a propulsion motor", id_);

        return;
    }

    if (frameMsg_->data[(size_t)RoverCanLib::Constant::eDataIndex::MSG_ID]
        == (size_t)RoverCanLib::Constant::eMsgId::PROPULSION_MOTOR_STATUS)
    {
        // Cast back msg from it's parent type to it's actual type (child) to be able to access the data member later on
        RoverCanLib::Msgs::PropulsionMotorStatus* msg
            = dynamic_cast<RoverCanLib::Msgs::PropulsionMotorStatus*>(_msgsMap.find(id_)->second);
        msg->parseMsg(frameMsg_, rclcpp::get_logger(LOGGER_NAME));

        if (RoverCanLib::Helpers::msgContentIsLastElement<RoverCanLib::Msgs::PropulsionMotorStatus>(frameMsg_))
        {
            uint8_t arrayIndex;

            switch (id_)
            {
                case (uint16_t)RoverCanLib::Constant::eDeviceId::FRONTLEFT_MOTOR:
                    arrayIndex = rover_msgs::msg::PropulsionMotor::FRONT_LEFT;
                    break;

                case (uint16_t)RoverCanLib::Constant::eDeviceId::FRONTRIGHT_MOTOR:
                    arrayIndex = rover_msgs::msg::PropulsionMotor::FRONT_RIGHT;
                    break;

                case (uint16_t)RoverCanLib::Constant::eDeviceId::REARLEFT_MOTOR:
                    arrayIndex = rover_msgs::msg::PropulsionMotor::REAR_LEFT;
                    break;

                case (uint16_t)RoverCanLib::Constant::eDeviceId::REARRIGHT_MOTOR:
                    arrayIndex = rover_msgs::msg::PropulsionMotor::REAR_RIGHT;
                    break;

                default: RCLCPP_FATAL(rclcpp::get_logger(LOGGER_NAME), "Shouldn't ever fall here, implementation error"); break;
            }
            msg_ROS_propMotor.current_speed[arrayIndex] = msg->data.currentSpeed;
            _pub_propulsionMotor->publish(msg_ROS_propMotor);
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                     "Received unexpected msg id: 0x%.2x Possible mismatch in library version between nodes",
                     frameMsg_->data[(size_t)RoverCanLib::Constant::eDataIndex::MSG_ID]);
    }
}

void CanMaster::CB_Can_Arm(uint16_t id_, const can_frame* frameMsg_)
{
    if (id_ != (size_t)RoverCanLib::Constant::eDeviceId::JL_CONTROLLER
        && id_ != (size_t)RoverCanLib::Constant::eDeviceId::J0_CONTROLLER
        && id_ != (size_t)RoverCanLib::Constant::eDeviceId::J1_CONTROLLER
        && id_ != (size_t)RoverCanLib::Constant::eDeviceId::J2_CONTROLLER
        && id_ != (size_t)RoverCanLib::Constant::eDeviceId::GRIPPER_TILT_CONTROLLER
        && id_ != (size_t)RoverCanLib::Constant::eDeviceId::GRIPPER_ROT_CONTROLLER
        && id_ != (size_t)RoverCanLib::Constant::eDeviceId::GRIPPER_CLOSE_CONTROLLER)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Provided device ID: 0x%.3x isn't a arm controller", id_);
        return;
    }

    if (frameMsg_->data[(size_t)RoverCanLib::Constant::eDataIndex::MSG_ID] == (size_t)RoverCanLib::Constant::eMsgId::ARM_STATUS)
    {
        // Cast back msg from it's parent type to it's actual type (child) to be able to access the data member later on
        RoverCanLib::Msgs::armStatus* msg = dynamic_cast<RoverCanLib::Msgs::armStatus*>(_msgsMap.find(id_)->second);
        msg->parseMsg(frameMsg_, rclcpp::get_logger(LOGGER_NAME));

        if (RoverCanLib::Helpers::msgContentIsLastElement<RoverCanLib::Msgs::armStatus>(frameMsg_))
        {
            uint8_t arrayIndex;

            switch (id_)
            {
                case (uint16_t)RoverCanLib::Constant::eDeviceId::JL_CONTROLLER: arrayIndex = rover_msgs::msg::ArmMsg::JL; break;

                case (uint16_t)RoverCanLib::Constant::eDeviceId::J0_CONTROLLER: arrayIndex = rover_msgs::msg::ArmMsg::J0; break;

                case (uint16_t)RoverCanLib::Constant::eDeviceId::J1_CONTROLLER: arrayIndex = rover_msgs::msg::ArmMsg::J1; break;

                case (uint16_t)RoverCanLib::Constant::eDeviceId::J2_CONTROLLER: arrayIndex = rover_msgs::msg::ArmMsg::J2; break;

                case (uint16_t)RoverCanLib::Constant::eDeviceId::GRIPPER_TILT_CONTROLLER:
                    arrayIndex = rover_msgs::msg::ArmMsg::GRIPPER_TILT;
                    break;

                case (uint16_t)RoverCanLib::Constant::eDeviceId::GRIPPER_ROT_CONTROLLER:
                    arrayIndex = rover_msgs::msg::ArmMsg::GRIPPER_ROT;
                    break;

                case (uint16_t)RoverCanLib::Constant::eDeviceId::GRIPPER_CLOSE_CONTROLLER:
                    arrayIndex = rover_msgs::msg::ArmMsg::GRIPPER_CLOSE;
                    break;

                default: RCLCPP_FATAL(rclcpp::get_logger(LOGGER_NAME), "Shouldn't ever fall here, implementation error"); break;
            }

            msg_ROS_arm.data[arrayIndex] = msg->data.currentSpeed;
            _pub_arm->publish(msg_ROS_arm);
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                     "Received unexpected msg id: 0x%.2x Possible mismatch in library version between nodes",
                     frameMsg_->data[(size_t)RoverCanLib::Constant::eDataIndex::MSG_ID]);
    }
}

void CanMaster::CB_Can_None(uint16_t dontUse0_, const can_frame* dontUse1_)
{
    REMOVE_UNUSED(dontUse0_);
    REMOVE_UNUSED(dontUse1_);
    return;
}

void CanMaster::CB_Can_GPS(uint16_t id_, const can_frame* frameMsg_)
{
    if (id_ != (size_t)RoverCanLib::Constant::eDeviceId::GPS)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Provided device ID: 0x%.3x isn't a GPS", id_);
        return;
    }

    if (frameMsg_->data[(size_t)RoverCanLib::Constant::eDataIndex::MSG_ID] == (size_t)RoverCanLib::Constant::eMsgId::GPS)
    {
        // Cast back msg from it's parent type to it's actual type (child) to be able to access the data member later on
        RoverCanLib::Msgs::GPS* msg = dynamic_cast<RoverCanLib::Msgs::GPS*>(_msgsMap.find(id_)->second);
        msg->parseMsg(frameMsg_, rclcpp::get_logger(LOGGER_NAME));

        if (RoverCanLib::Helpers::msgContentIsLastElement<RoverCanLib::Msgs::GPS>(frameMsg_))
        {
            msg_ROS_gps.latitude = msg->data.latitude;
            msg_ROS_gps.longitude = msg->data.longitude;
            _pub_gps->publish(msg_ROS_gps);
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                     "Received unexpected msg id: 0x%.2x Possible mismatch in library version between nodes",
                     frameMsg_->data[(size_t)RoverCanLib::Constant::eDataIndex::MSG_ID]);
    }
}

void CanMaster::CB_Can_Compass(uint16_t id_, const can_frame* frameMsg_)
{
    if (id_ != (size_t)RoverCanLib::Constant::eDeviceId::COMPASS)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Provided device ID: 0x%.3x isn't a compass", id_);
        return;
    }

    if (frameMsg_->data[(size_t)RoverCanLib::Constant::eDataIndex::MSG_ID] == (size_t)RoverCanLib::Constant::eMsgId::COMPASS)
    {
        // Cast back msg from it's parent type to it's actual type (child) to be able to access the data member later on
        RoverCanLib::Msgs::Compass* msg = dynamic_cast<RoverCanLib::Msgs::Compass*>(_msgsMap.find(id_)->second);
        msg->parseMsg(frameMsg_, rclcpp::get_logger(LOGGER_NAME));

        if (RoverCanLib::Helpers::msgContentIsLastElement<RoverCanLib::Msgs::Compass>(frameMsg_))
        {
            msg_ROS_compass.heading = msg->data.heading;
            msg_ROS_compass.pitch = msg->data.pitch;
            _pub_compass->publish(msg_ROS_compass);
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                     "Received unexpected msg id: 0x%.2x Possible mismatch in library version between nodes",
                     frameMsg_->data[(size_t)RoverCanLib::Constant::eDataIndex::MSG_ID]);
    }
}

// =============================================================================
//  Sub callbacks
void CanMaster::CB_ROS_propulsionMotor(const rover_msgs::msg::PropulsionMotor::SharedPtr rosMsg_)
{
    if (!_timer_motorCmdSend.isDone())
    {
        return;
    }

    RoverCanLib::Msgs::PropulsionMotorCmd canMsg;

    for (uint8_t arrayIndex = 0; arrayIndex < rosMsg_->enable.size(); arrayIndex++)
    {
        uint16_t deviceId;

        switch (arrayIndex)
        {
            case rover_msgs::msg::PropulsionMotor::FRONT_LEFT:
                deviceId = (uint16_t)RoverCanLib::Constant::eDeviceId::FRONTLEFT_MOTOR;
                break;

            case rover_msgs::msg::PropulsionMotor::FRONT_RIGHT:
                deviceId = (uint16_t)RoverCanLib::Constant::eDeviceId::FRONTRIGHT_MOTOR;
                break;

            case rover_msgs::msg::PropulsionMotor::REAR_LEFT:
                deviceId = (uint16_t)RoverCanLib::Constant::eDeviceId::REARLEFT_MOTOR;
                break;

            case rover_msgs::msg::PropulsionMotor::REAR_RIGHT:
                deviceId = (uint16_t)RoverCanLib::Constant::eDeviceId::REARRIGHT_MOTOR;
                break;

            default: RCLCPP_FATAL(rclcpp::get_logger(LOGGER_NAME), "Shouldn't ever fall here, implementation error"); break;
        }

        canMsg.data.enable = rosMsg_->enable[arrayIndex];
        canMsg.data.targetSpeed = rosMsg_->target_speed[arrayIndex];
        canMsg.data.closeLoop = rosMsg_->close_loop[arrayIndex];

        canMsg.sendMsg((RoverCanLib::Constant::eDeviceId)deviceId, _canSocket, rclcpp::get_logger(LOGGER_NAME));
    }
}

void CanMaster::CB_ROS_Arm(const rover_msgs::msg::ArmMsg::SharedPtr rosMsg_)
{
    if (!_timer_armCmdSend.isDone())
    {
        return;
    }

    RoverCanLib::Msgs::armCmd canMsg;

    for (uint8_t arrayIndex = 0; arrayIndex < rosMsg_->data.size(); arrayIndex++)
    {
        uint16_t deviceId;

        switch (arrayIndex)
        {
            case rover_msgs::msg::ArmMsg::JL: deviceId = (uint16_t)RoverCanLib::Constant::eDeviceId::JL_CONTROLLER; break;

            case rover_msgs::msg::ArmMsg::J0: deviceId = (uint16_t)RoverCanLib::Constant::eDeviceId::J0_CONTROLLER; break;

            case rover_msgs::msg::ArmMsg::J1: deviceId = (uint16_t)RoverCanLib::Constant::eDeviceId::J1_CONTROLLER; break;

            case rover_msgs::msg::ArmMsg::J2: deviceId = (uint16_t)RoverCanLib::Constant::eDeviceId::J2_CONTROLLER; break;

            case rover_msgs::msg::ArmMsg::GRIPPER_TILT:
                deviceId = (uint16_t)RoverCanLib::Constant::eDeviceId::GRIPPER_TILT_CONTROLLER;
                break;

            case rover_msgs::msg::ArmMsg::GRIPPER_ROT:
                deviceId = (uint16_t)RoverCanLib::Constant::eDeviceId::GRIPPER_ROT_CONTROLLER;
                break;

            case rover_msgs::msg::ArmMsg::GRIPPER_CLOSE:
                deviceId = (uint16_t)RoverCanLib::Constant::eDeviceId::GRIPPER_CLOSE_CONTROLLER;
                break;

            default: RCLCPP_FATAL(rclcpp::get_logger(LOGGER_NAME), "Shouldn't ever fall here, implementation error"); break;
        }

        canMsg.data.targetSpeed = rosMsg_->data[arrayIndex];
        canMsg.sendMsg((RoverCanLib::Constant::eDeviceId)deviceId, _canSocket, rclcpp::get_logger(LOGGER_NAME));
    }
}

void CanMaster::CB_ROS_cameraControl(const rover_msgs::msg::CameraControl::SharedPtr rosMsg_)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Not implemented yet");
    REMOVE_UNUSED(rosMsg_);
}

void CanMaster::CB_ROS_lightControl(const rover_msgs::msg::LightControl::SharedPtr rosMsg_)
{
    RoverCanLib::Msgs::lightControl msg;
    msg.data.enable = rosMsg_->enable[rover_msgs::msg::LightControl::LIGHT];
    msg.sendMsg(RoverCanLib::Constant::eDeviceId::LIGHTS, _canSocket, rclcpp::get_logger(LOGGER_NAME));

    msg.data.enable = rosMsg_->enable[rover_msgs::msg::LightControl::LIGHT_INFRARED];
    msg.sendMsg(RoverCanLib::Constant::eDeviceId::INFRARED_LIGHTS, _canSocket, rclcpp::get_logger(LOGGER_NAME));
}

void CanMaster::CB_ROS_cameraPano(const rover_msgs::msg::CameraAngle::SharedPtr rosMsg_)
{
    RoverCanLib::Msgs::CamPan msg;
    msg.data.servoPosition = rosMsg_->angle;

    msg.sendMsg(RoverCanLib::Constant::eDeviceId::CAMERA_PAN, _canSocket, rclcpp::get_logger(LOGGER_NAME));
}
