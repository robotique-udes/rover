// Network socket stuff
#include <net/if.h>
#include <fcntl.h>

// ROS
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/can_device_status.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "rover_msgs/msg/camera_control.hpp"
#include "rover_msgs/msg/light_control.hpp"

// RoverCanLib
#include "rover_can_lib/config.hpp"
#include "rover_can_lib/can_device.hpp"
#include "rover_can_lib/msgs/heartbeat.hpp"
#include "rover_can_lib/msgs/propulsion_motor_cmd.hpp"
#include "rover_can_lib/msgs/propulsion_motor_status.hpp"
#include "rover_can_lib/msgs/cam_control.hpp"
#include "rover_can_lib/msgs/cam_control_a2.hpp"
#include "rover_can_lib/msgs/light_control.hpp"

#define LOGGER_NAME "CanMasterNode"

// Forward declarations
int createSocket(const char *canNetworkName_);
volatile sig_atomic_t shutdownFlag = 0;
void signal_handler(int signo);

Timer<unsigned long, millis> motorCmdSendTimer(50); // 10 Hz

// =============================================================================
//  Per devices msg object
// PDB
RoverCanLib::Msgs::camControl msg_CAN_CAMERA_A2;
RoverCanLib::Msgs::camControl msg_CAN_CAMERA_R1M_1;
RoverCanLib::Msgs::camControl msg_CAN_CAMERA_R1M_2;
RoverCanLib::Msgs::camControl msg_CAN_CAMERA_R1M_3;
RoverCanLib::Msgs::lightControl msg_CAN_LIGHTS;

// Motors
RoverCanLib::Msgs::PropulsionMotorStatus msg_CAN_FrontLeft;
RoverCanLib::Msgs::PropulsionMotorStatus msg_CAN_FrontRight;
RoverCanLib::Msgs::PropulsionMotorStatus msg_CAN_RearLeft;
RoverCanLib::Msgs::PropulsionMotorStatus msg_CAN_RearRight;
// =============================================================================

// =============================================================================
//  Global ROS msg object
rover_msgs::msg::PropulsionMotor msg_ROS_propMotor;
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
    Timer<uint64_t, millis> _timerHeartbeat = Timer<uint64_t, millis>((uint64_t)(1000.0f / RoverCanLib::Constant::HEARTBEAT_FREQ));
    Chrono<uint64_t, millis> chonoCanWatchdog;
    rclcpp::TimerBase::SharedPtr _timerLoop;
    std::unordered_map<size_t, CanDevice> _deviceMap;
    std::unordered_map<size_t, RoverCanLib::Msgs::Msg *> _msgsMap;

    // =========================================================================
    //  Device publishers
    rclcpp::Publisher<rover_msgs::msg::CanDeviceStatus>::SharedPtr _pub_canStatus;
    rclcpp::Publisher<rover_msgs::msg::PropulsionMotor>::SharedPtr _pub_propulsionMotor;

    // =========================================================================

    // =========================================================================
    //  Device subscriber
    rclcpp::Subscription<rover_msgs::msg::PropulsionMotor>::SharedPtr _sub_propulsionMotor;
    rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr _sub_cameras;
    rclcpp::Subscription<rover_msgs::msg::LightControl>::SharedPtr _sub_lights;
    // =========================================================================

    // =========================================================================
    //  Devices callbacks
    void CB_Can_None(uint16_t dontUse0_, const can_frame *dontUse1_);
    void CB_Can_PropulsionMotor(uint16_t id_, const can_frame *frameMsg);
    // =========================================================================

    // =========================================================================
    //  Ros callbacks
    void CB_ROS_propulsionMotor(const rover_msgs::msg::PropulsionMotor::SharedPtr msg);
    void CB_ROS_cameraControl(const rover_msgs::msg::CameraControl::SharedPtr rosMsg);
    void CB_ROS_lightControl(const rover_msgs::msg::LightControl::SharedPtr rosMsg);
    // =========================================================================
};

int main(int argc, char *argv[])
{
    signal(SIGINT, signal_handler); // Makes CTRL+C work

    while (!shutdownFlag)
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
void signal_handler(int signo)
{
    RCLCPP_INFO(rclcpp::get_logger("[SHUTDOWN]"), "Shutdown asked by user, exiting...");
    (void)signo;
    shutdownFlag = 1;
}

// Returns a socket on success or < 0 on error
int createSocket(const char *canNetworkName_)
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
    if (bind(canSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
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
CanMaster::CanMaster(int canSocket_) : Node("can_master")
{
    _canSocket = canSocket_;

    // =========================================================================
    //  Device publishers actual creation
    _pub_canStatus = this->create_publisher<rover_msgs::msg::CanDeviceStatus>("/rover/can/device_status", 1);
    _pub_propulsionMotor = this->create_publisher<rover_msgs::msg::PropulsionMotor>("/rover/drive_train/status/prop_motor", 1);
    // =========================================================================

    // Add messages type to msgsMap
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::CAMERA_A2] = &msg_CAN_CAMERA_A2;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::CAMERA_R1M_1] = &msg_CAN_CAMERA_R1M_1;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::CAMERA_R1M_2] = &msg_CAN_CAMERA_R1M_2;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::CAMERA_R1M_3] = &msg_CAN_CAMERA_R1M_3;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::LIGHTS] = &msg_CAN_LIGHTS;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::FRONTLEFT_MOTOR] = &msg_CAN_FrontLeft;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::FRONTRIGHT_MOTOR] = &msg_CAN_FrontRight;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::REARLEFT_MOTOR] = &msg_CAN_RearLeft;
    _msgsMap[(size_t)RoverCanLib::Constant::eDeviceId::REARRIGHT_MOTOR] = &msg_CAN_RearRight;

    // =========================================================================
    //  Devices objects constructors

    // PDB
    _deviceMap.emplace((size_t)RoverCanLib::Constant::eDeviceId::PDB_CONTROLLER, CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::PDB_CONTROLLER, this, &CanMaster::CB_Can_None, _pub_canStatus));
    _deviceMap.emplace((size_t)RoverCanLib::Constant::eDeviceId::CAMERA_A2, CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::CAMERA_A2, this, &CanMaster::CB_Can_None, _pub_canStatus));
    _deviceMap.emplace((size_t)RoverCanLib::Constant::eDeviceId::CAMERA_R1M_1, CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::CAMERA_R1M_1, this, &CanMaster::CB_Can_None, _pub_canStatus));
    _deviceMap.emplace((size_t)RoverCanLib::Constant::eDeviceId::CAMERA_R1M_2, CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::CAMERA_R1M_2, this, &CanMaster::CB_Can_None, _pub_canStatus));
    _deviceMap.emplace((size_t)RoverCanLib::Constant::eDeviceId::CAMERA_R1M_3, CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::CAMERA_R1M_3, this, &CanMaster::CB_Can_None, _pub_canStatus));
    _deviceMap.emplace((size_t)RoverCanLib::Constant::eDeviceId::LIGHTS, CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::CAMERA_R1M_3, this, &CanMaster::CB_Can_None, _pub_canStatus));

    // Motor Drive
    _deviceMap.emplace((size_t)RoverCanLib::Constant::eDeviceId::FRONTLEFT_MOTOR, CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::FRONTLEFT_MOTOR, this, &CanMaster::CB_Can_PropulsionMotor, _pub_canStatus));
    _deviceMap.emplace((size_t)RoverCanLib::Constant::eDeviceId::FRONTRIGHT_MOTOR, CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::FRONTRIGHT_MOTOR, this, &CanMaster::CB_Can_PropulsionMotor, _pub_canStatus));
    _deviceMap.emplace((size_t)RoverCanLib::Constant::eDeviceId::REARLEFT_MOTOR, CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::REARLEFT_MOTOR, this, &CanMaster::CB_Can_PropulsionMotor, _pub_canStatus));
    _deviceMap.emplace((size_t)RoverCanLib::Constant::eDeviceId::REARRIGHT_MOTOR, CanDevice((uint16_t)RoverCanLib::Constant::eDeviceId::REARRIGHT_MOTOR, this, &CanMaster::CB_Can_PropulsionMotor, _pub_canStatus));
    // =========================================================================

    this->askStateCanDevices();

    // =========================================================================
    //  Topic subscriber actual creation
    // _sub_propulsionMotor = this->create_subscription<rover_msgs::msg::PropulsionMotor>("/rover/drive_train/cmd/out/motors", 1, std::bind(&CanMaster::CB_ROS_PropulsionMotor, this, std::placeholders::_1));
    _sub_propulsionMotor = this->create_subscription<rover_msgs::msg::PropulsionMotor>("/rover/drive_train/cmd/in/teleop", 1, std::bind(&CanMaster::CB_ROS_propulsionMotor, this, std::placeholders::_1));
    _sub_cameras = this->create_subscription<rover_msgs::msg::CameraControl>("/TODO/CAM_TOPIC", 1, std::bind(&CanMaster::CB_ROS_cameraControl, this, std::placeholders::_1));
    _sub_lights = this->create_subscription<rover_msgs::msg::LightControl>("/rover/auxiliary/lights/status", 1, std::bind(&CanMaster::CB_ROS_lightControl, this, std::placeholders::_1));
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
    if (chonoCanWatchdog.getTime() > RoverCanLib::Constant::WATCHDOG_TIMEOUT_MS)
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
            chonoCanWatchdog.restart();

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

    return msgErrorState.sendMsg(RoverCanLib::Constant::eDeviceId::MASTER_COMPUTER_UNIT, _canSocket, rclcpp::get_logger(LOGGER_NAME));
}

// =============================================================================
//  Devices callbacks
void CanMaster::CB_Can_PropulsionMotor(uint16_t id_, const can_frame *frameMsg)
{
    if (id_ != (size_t)RoverCanLib::Constant::eDeviceId::FRONTLEFT_MOTOR &&
        id_ != (size_t)RoverCanLib::Constant::eDeviceId::FRONTRIGHT_MOTOR &&
        id_ != (size_t)RoverCanLib::Constant::eDeviceId::REARLEFT_MOTOR &&
        id_ != (size_t)RoverCanLib::Constant::eDeviceId::REARRIGHT_MOTOR)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                     "Provided device ID: 0x%.3x isn't a propulsion motor",
                     id_);

        return;
    }

    if (frameMsg->data[(size_t)RoverCanLib::Constant::eDataIndex::MSG_ID] == (size_t)RoverCanLib::Constant::eMsgId::PROPULSION_MOTOR_STATUS)
    {
        // Cast back msg from it's parent type to it's actual type (child) to be able to access the data member later on
        RoverCanLib::Msgs::PropulsionMotorStatus *msg = dynamic_cast<RoverCanLib::Msgs::PropulsionMotorStatus *>(_msgsMap.find(id_)->second);
        msg->parseMsg(frameMsg, rclcpp::get_logger(LOGGER_NAME));

        if (RoverCanLib::Helpers::msgContentIsLastElement<RoverCanLib::Msgs::PropulsionMotorStatus>(frameMsg))
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

            default:
                RCLCPP_FATAL(rclcpp::get_logger(LOGGER_NAME), "Shouldn't ever fall here, implementation error");
                break;
            }
            msg_ROS_propMotor.current_speed[arrayIndex] = msg->data.currentSpeed;
            _pub_propulsionMotor->publish(msg_ROS_propMotor);
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                     "Received unexpected msg id: 0x%.2x Possible mismatch in library version between nodes",
                     frameMsg->data[(size_t)RoverCanLib::Constant::eDataIndex::MSG_ID]);
    }
}

void CanMaster::CB_Can_None(uint16_t dontUse0_, const can_frame *dontUse1_)
{
    REMOVE_UNUSED(dontUse0_);
    REMOVE_UNUSED(dontUse1_);
    return;
}

// =============================================================================
//  Sub callbacks
void CanMaster::CB_ROS_propulsionMotor(const rover_msgs::msg::PropulsionMotor::SharedPtr rosMsg)
{
    if (!motorCmdSendTimer.isDone())
    {
        return;
    }

    RoverCanLib::Msgs::PropulsionMotorCmd canMsg;

    for (uint8_t arrayIndex = 0; arrayIndex < rosMsg->enable.size(); arrayIndex++)
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

        default:
            RCLCPP_FATAL(rclcpp::get_logger(LOGGER_NAME), "Shouldn't ever fall here, implementation error");
            break;
        }

        canMsg.data.enable = rosMsg->enable[arrayIndex];
        canMsg.data.targetSpeed = rosMsg->target_speed[arrayIndex];
        canMsg.data.closeLoop = rosMsg->close_loop[arrayIndex];

        canMsg.sendMsg((RoverCanLib::Constant::eDeviceId)deviceId, _canSocket, rclcpp::get_logger(LOGGER_NAME));
    }
}

void CanMaster::CB_ROS_cameraControl(const rover_msgs::msg::CameraControl::SharedPtr rosMsg)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Not implemented yet");

    REMOVE_UNUSED(rosMsg);
    // RoverCanLib::Msgs::PropulsionMotorCmd canMsg;

    // for (uint8_t arrayIndex = 0; arrayIndex < rosMsg->enable.size(); arrayIndex++)
    // {
    //     uint16_t deviceId;

    //     switch (arrayIndex)
    //     {
    //     case rover_msgs::msg::PropulsionMotor::FRONT_LEFT:
    //         deviceId = (uint16_t)RoverCanLib::Constant::eDeviceId::FRONTLEFT_MOTOR;
    //         break;

    //     case rover_msgs::msg::PropulsionMotor::FRONT_RIGHT:
    //         deviceId = (uint16_t)RoverCanLib::Constant::eDeviceId::FRONTRIGHT_MOTOR;
    //         break;

    //     case rover_msgs::msg::PropulsionMotor::REAR_LEFT:
    //         deviceId = (uint16_t)RoverCanLib::Constant::eDeviceId::REARLEFT_MOTOR;
    //         break;

    //     case rover_msgs::msg::PropulsionMotor::REAR_RIGHT:
    //         deviceId = (uint16_t)RoverCanLib::Constant::eDeviceId::REARRIGHT_MOTOR;
    //         break;

    //     default:
    //         RCLCPP_FATAL(rclcpp::get_logger(LOGGER_NAME), "Shouldn't ever fall here, implementation error");
    //         break;
    //     }

    //     canMsg.data.enable = rosMsg->enable[arrayIndex];
    //     canMsg.data.targetSpeed = rosMsg->target_speed[arrayIndex];
    //     canMsg.data.closeLoop = rosMsg->close_loop[arrayIndex];

    //     canMsg.sendMsg((RoverCanLib::Constant::eDeviceId)deviceId, _canSocket, rclcpp::get_logger(LOGGER_NAME));
    // }
}

void CanMaster::CB_ROS_lightControl(const rover_msgs::msg::LightControl::SharedPtr rosMsg)
{
    printf("Sending msg");
    RoverCanLib::Msgs::lightControl msg;
    msg.data.enable = rosMsg->enable[rover_msgs::msg::LightControl::LIGHT];
    msg.sendMsg(RoverCanLib::Constant::eDeviceId::LIGHTS, _canSocket, rclcpp::get_logger(LOGGER_NAME));

    msg.data.enable = rosMsg->enable[rover_msgs::msg::LightControl::LIGHT_INFRARED];
    msg.sendMsg(RoverCanLib::Constant::eDeviceId::INFRARED_LIGHTS, _canSocket, rclcpp::get_logger(LOGGER_NAME));
}
