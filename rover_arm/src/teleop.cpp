#include "rclcpp/rclcpp.hpp"

#include "rover_msgs/msg/arm_msg.hpp"
#include "rover_msgs/msg/joy.hpp"

#include "rovus_lib/timer.hpp"
#include "rovus_lib/macros.h"
#include "arm_configuration.hpp"
#include "keybinding.hpp"

#include <joint_controller.hpp>
#include <gripper_controller.hpp>
#include <cartesian_controller.hpp>

class Teleop : public rclcpp::Node
{
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

    JointController _jointController;
    // GripperController _gripperController;
    CartesianController _cartesianController;
    JoyController _joyController;

    eControlMode _controlMode = eControlMode::JOINT;

    std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> _jointPositions;

  public:
    Teleop():
        rclcpp::Node("teleop_node"),
        _jointController({eJointIndex::JL, eJointIndex::J1, eJointIndex::J2, eJointIndex::GRIPPER_TILT}),
        // _gripperController({eJointIndex::GRIPPER_TILT, eJointIndex::GRIPPER_ROT}),
        _cartesianController({eJointIndex::JL, eJointIndex::J1, eJointIndex::J2, eJointIndex::GRIPPER_TILT}),
        _joyController()
    {
        _subJoyArm = this->create_subscription<rover_msgs::msg::Joy>("/rover/arm/joy",
                                                                     1,
                                                                     [this](const rover_msgs::msg::Joy& joyMsg_)
                                                                     {
                                                                         this->joy_CB(joyMsg_);
                                                                     });
        _subArmPositions = this->create_subscription<rover_msgs::msg::ArmMsg>("/rover/arm/status/current_positions",
                                                                              1,
                                                                              [this](const rover_msgs::msg::ArmMsg& armMsg_)
                                                                              {
                                                                                  this->position_CB(armMsg_);
                                                                              });

        _pubArmCmd = this->create_publisher<rover_msgs::msg::ArmMsg>("/rover/arm/cmd/goal_speed", 1);
    }

    void joy_CB(const rover_msgs::msg::Joy& joyMsg_)
    {
        const uint8_t joyMsgSize = joyMsg_.joy_data.size();
        std::array<float, TO_UNDERLYING(eJoyInput::eLAST)> joyArray = {};
        std::copy_n(joyMsg_.joy_data.begin(), joyMsgSize, joyArray.begin());
        rover_msgs::msg::ArmMsg armMsg;

        // TOGGLE CONTROL MODE
        if (_joyController.isSelected(joyArray[TO_UNDERLYING(KEYBINDINGS::EMILE::CARTESIAN::TOGGLE_CARTESIAN)],
                                      KEYBINDINGS::EMILE::CARTESIAN::TOGGLE_CARTESIAN))
        {
            if (_controlMode == eControlMode::JOINT)
            {
                _controlMode = eControlMode::CARTESIAN;
                RCLCPP_INFO(this->get_logger(), "Control mode is now CARTESIAN");
            }
            else if ((_controlMode == eControlMode::CARTESIAN))
            {
                _controlMode = eControlMode::JOINT;
                RCLCPP_INFO(this->get_logger(), "Control mode is now JOINT");
            }
        }

        // JOINT CONTROL -- DEFAULT MODE
        if (_controlMode == eControlMode::JOINT)
        {
            if (_joyController.isSelected(joyArray[TO_UNDERLYING(KEYBINDINGS::EMILE::JOINT::JOINT_SELECT_INC)],
                                          KEYBINDINGS::EMILE::JOINT::JOINT_SELECT_INC))
            {
                _jointController.setControlledJoint(KEYBINDINGS::EMILE::JOINT::JOINT_SELECT_INC);
            }
            if (_joyController.isSelected(joyArray[TO_UNDERLYING(KEYBINDINGS::EMILE::JOINT::JOINT_SELECT_DEC)],
                                          KEYBINDINGS::EMILE::JOINT::JOINT_SELECT_DEC))
            {
                _jointController.setControlledJoint(KEYBINDINGS::EMILE::JOINT::JOINT_SELECT_DEC);
            }

            armMsg.data = _jointController.getJointCmdFromInput(joyArray);
        }

        // CARTESIAN CONTROL
        else if (_controlMode == eControlMode::CARTESIAN)
        {
            _cartesianController.getJointPositions(_jointPositions);
            armMsg.data = _cartesianController.getJointCmdFromInput(joyArray);

            if (_joyController.isSelected(joyArray[TO_UNDERLYING(KEYBINDINGS::EMILE::CARTESIAN::RECORD)],
                                          KEYBINDINGS::EMILE::CARTESIAN::RECORD))
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

            if (_joyController.isSelected(joyArray[TO_UNDERLYING(KEYBINDINGS::EMILE::CARTESIAN::CREATE_PLAN)],
                                          KEYBINDINGS::EMILE::CARTESIAN::CREATE_PLAN))
            {
                if (_cartesianController.getRecordedPoints() != TO_UNDERLYING(CartesianController::eCartesianCoord::eLAST))
                {
                    RCLCPP_WARN(this->get_logger(), "Cannot create plan since not enough points have been gathered");
                }
                else
                {
                    if (_cartesianController.applyPlan())
                    {
                        RCLCPP_INFO(this->get_logger(), "Applying plan");
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "Unapplying");
                    }
                }
            }
        }

        // // GRIPPER CONTROL
        // if (_joyController.isPressed(joyArray[TO_UNDERLYING(KEYBINDINGS::EMILE::GRIPPER::ACTIVATE_GRIPPER)]))
        // {
        //     armMsg.data = _gripperController.setCmd(joyArray);
        // }

        _pubArmCmd->publish(armMsg);
    }

    void position_CB(const rover_msgs::msg::ArmMsg& armMsg_)
    {
        std::copy_n(armMsg_.data.begin(), TO_UNDERLYING(eJointIndex::eLAST), _jointPositions.begin());
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
}
