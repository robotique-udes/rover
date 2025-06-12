
#include "arm_configuration.hpp"
#include "keybinding.hpp"
#include "joint_controller.hpp"
#include "cartesian_controller.hpp"

#include <rover_msgs/msg/arm_msg.hpp>
#include <rover_msgs/msg/joy.hpp>

#include <rover_lib2/helpers/time.hpp>
#include <rover_lib2/helpers/loop_timer.hpp>
#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>

#include <rclcpp/rclcpp.hpp>

class Teleop : public rclcpp::Node
{
    static constexpr const char* TOPIC_JOY_ARM = "/base/joy/arm";
    static constexpr const char* TOPIC_ARM_STATUS = "/rover/arm/joints_status";
    static constexpr const char* TOPIC_ARM_CMD = "/rover/arm/joints_cmd";

  public:
    enum class eControlMode : size_t
    {
        JOINT = 0,
        CARTESIAN = 1
    };

  private:
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _subJoyArm;
    rclcpp::Subscription<rover_msgs::msg::ArmMsg>::SharedPtr _subArmPositions;
    rclcpp::Publisher<rover_msgs::msg::ArmMsg>::SharedPtr _pubArmCmd;

    JoyManager _joyManager;
    JointController _jointController;
    CartesianController _cartesianController;

    eControlMode _controlMode = eControlMode::JOINT;

    std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> _jointPositions;

  public:
    Teleop():
        rclcpp::Node("teleop_node"),
        _jointController(_joyManager),
        _cartesianController(_joyManager)
    {
        _subJoyArm = this->create_subscription<rover_msgs::msg::Joy>(TOPIC_JOY_ARM,
                                                                     QOS_DEFAULT,
                                                                     [this](const rover_msgs::msg::Joy& joyMsg_)
                                                                     {
                                                                         this->joy_CB(joyMsg_);
                                                                     });
        _subArmPositions = this->create_subscription<rover_msgs::msg::ArmMsg>(TOPIC_ARM_STATUS,
                                                                              QOS_DEFAULT,
                                                                              [this](const rover_msgs::msg::ArmMsg& armMsg_)
                                                                              {
                                                                                  this->position_CB(armMsg_);
                                                                              });

        _pubArmCmd = this->create_publisher<rover_msgs::msg::ArmMsg>(TOPIC_ARM_CMD, QOS_DEFAULT);
    }

    void joy_CB(const rover_msgs::msg::Joy& joyMsg_)
    {
        const size_t joyMsgSize = joyMsg_.joy_data.size();
        std::array<float, TO_UNDERLYING(eJoyInput::eLAST)> joyArray = {};
        std::copy_n(joyMsg_.joy_data.begin(), joyMsgSize, joyArray.begin());
        rover_msgs::msg::ArmMsg armMsg;

        _joyManager.updateJoyArray(joyArray);

        // TOGGLE CONTROL MODE
        if (_joyManager.isTriggered(KEYBINDINGS::CARTESIAN::TOGGLE_CARTESIAN))
        {
            if (_controlMode == eControlMode::JOINT)
            {
                _controlMode = eControlMode::CARTESIAN;
                RCLCPP_INFO(this->get_logger(), "Control mode is now CARTESIAN");
            }
            else if (_controlMode == eControlMode::CARTESIAN)
            {
                _controlMode = eControlMode::JOINT;
                RCLCPP_INFO(this->get_logger(), "Control mode is now JOINT");
            }
        }

        // JOINT CONTROL -- DEFAULT MODE
        if (_controlMode == eControlMode::JOINT)
        {
            if (_joyManager.isTriggered(KEYBINDINGS::JOINT::JOINT_SELECT_INC))
            {
                _jointController.setControlledJoint(KEYBINDINGS::JOINT::JOINT_SELECT_INC);
            }
            if (_joyManager.isTriggered(KEYBINDINGS::JOINT::JOINT_SELECT_DEC))
            {
                _jointController.setControlledJoint(KEYBINDINGS::JOINT::JOINT_SELECT_DEC);
            }

            armMsg.target_speed = _jointController.getJointCmdFromInput(joyArray);
        }

        // CARTESIAN CONTROL
        else if (_controlMode == eControlMode::CARTESIAN)
        {
            _cartesianController.getJointPositions(_jointPositions);
            armMsg.target_speed = _cartesianController.getJointCmdFromInput(joyArray);

            if (_joyManager.isTriggered(KEYBINDINGS::CARTESIAN::RECORD))
            {
                if (_cartesianController.getRecordedPoints() == TO_UNDERLYING(CartesianController::eCartesianCoord::eLAST))
                {
                    RCLCPP_WARN(this->get_logger(), "No more points can be recorded. Create a plan or clear all points");
                }
                else
                {
                    _cartesianController.addPoint(_cartesianController.getEndEffectorPose(_jointPositions));
                    RCLCPP_INFO(this->get_logger(), "Point has been added to array");
                }
            }

            if (_joyManager.isTriggered(KEYBINDINGS::CARTESIAN::CREATE_PLAN))
            {
                if (_cartesianController.getRecordedPoints() != TO_UNDERLYING(CartesianController::eCartesianCoord::eLAST))
                {
                    RCLCPP_WARN(this->get_logger(), "Cannot create plan since not enough points have been gathered");
                }
                else if (_cartesianController.applyPlan())
                {
                    RCLCPP_INFO(this->get_logger(), "Applying plan");
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Unapplying");
                }
            }
        }

        // // GRIPPER CONTROL
        // if (_joyManager.isPressed(KEYBINDINGS::GRIPPER::ACTIVATE_GRIPPER)]))
        // {
        //     armMsg.data = _gripperController.setCmd(joyArray);
        // }

        _pubArmCmd->publish(armMsg);
    }

    void position_CB(const rover_msgs::msg::ArmMsg& armMsg_)
    {
        std::copy_n(armMsg_.target_speed.begin(), TO_UNDERLYING(eJointIndex::eLAST), _jointPositions.begin());
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
}
