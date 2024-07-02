#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"
#include "rover_msgs/msg/arm_cmd.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "rovus_lib/macros.h"

class Teleop : public rclcpp::Node
{
public:
    Teleop();

private:

    // Private members
    //  =========================================================================
    // float _linkj0Toj1 = 529.5715f;
    // float _linkj1ToJ2 = 608.45785f;
    // float _endEffectorLength = 210.1945f;

    float _linearControl;
    float _j0Control;
    float _j1Control;
    float _j2Control;
    float _gripperUD;
    float _gripperLR;
    float _gripperO;
    float _gripperC;

    float _deadmanSwitch;
    float _gripperMode;

    float _forwardKinematic = 1.0f;
    float _inverseKinematic;

    float _speedControl = 0.0017453f; // 10 deg/s - (0.1745 rad/s)

    // Set Constrain values
    //  =========================================================================
    float _currentLinear;
    float _currentJ0;
    float _currentJ1;
    float _currentJ2;
    float _currentGripperLR;
    float _currentGripperUD;
    float _currentGripperOC;

    // Private methods
    //  =========================================================================
    void joyCallback(const rover_msgs::msg::Joy::SharedPtr joyMsg)
    {
        rover_msgs::msg::ArmCmd armMsg;

        // Mode selection
        // =========================================================================
        _deadmanSwitch = joyMsg->joy_data[rover_msgs::msg::Joy::L1];
        _gripperMode = joyMsg->joy_data[rover_msgs::msg::Joy::R1];

        // Arm controls
        // =========================================================================
        _linearControl = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_SIDE];
        _j0Control = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE];
        _j1Control = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT];
        _j2Control = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT];

        // Gripper controls
        // =========================================================================
        _gripperUD = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT];
        _gripperLR = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE];
        _gripperO = joyMsg->joy_data[rover_msgs::msg::Joy::R2];
        _gripperC = joyMsg->joy_data[rover_msgs::msg::Joy::L2];

        // Initialize controls to previous values
        // =========================================================================
        armMsg.position[rover_msgs::msg::ArmCmd::LINEAR] = _currentLinear;
        armMsg.position[rover_msgs::msg::ArmCmd::J0] = _currentJ0;
        armMsg.position[rover_msgs::msg::ArmCmd::J1] = _currentJ1;
        armMsg.position[rover_msgs::msg::ArmCmd::J2] = _currentJ2;
        armMsg.position[rover_msgs::msg::ArmCmd::GRIPPERLR] = _currentGripperLR;
        armMsg.position[rover_msgs::msg::ArmCmd::GRIPPERUD] = _currentGripperUD;
        armMsg.position[rover_msgs::msg::ArmCmd::GRIPPEROC] = _currentGripperOC;

        // Control logic
        // =========================================================================
        if (_deadmanSwitch)
        {
            if (_forwardKinematic)
            {
                if (_gripperMode)
                {
                    _currentGripperLR += _gripperLR * _speedControl;
                    _currentGripperUD += _gripperUD * _speedControl;
                    _currentGripperOC += (_gripperO - _gripperC) * _speedControl;
                }
                else
                {
                    _currentLinear += _linearControl * _speedControl;
                    _currentJ0 += _j0Control * _speedControl * -1.0f;
                    _currentJ1 += _j1Control * _speedControl;
                    _currentJ2 += _j2Control * _speedControl;
                }
            }
            else if (_inverseKinematic)
            {
                RCLCPP_ERROR(this->get_logger(), "Inverse kinematics mode NOT YET AVAILABLE");
            }
        }

        _pub_arm_teleop_in->publish(armMsg);
    }

    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_joy_arm;
    rclcpp::Publisher<rover_msgs::msg::ArmCmd>::SharedPtr _pub_arm_teleop_in;
};

// Teleop class constructor
// =========================================================================
Teleop::Teleop() : Node("teleop")
{
    _sub_joy_arm = this->create_subscription<rover_msgs::msg::Joy>("/rover/arm/joy",
                                                                   1,
                                                                   std::bind(&Teleop::joyCallback, this, std::placeholders::_1));
    _pub_arm_teleop_in = this->create_publisher<rover_msgs::msg::ArmCmd>("/rover/arm/cmd/in/teleop", 1);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
}