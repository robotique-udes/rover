
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
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_options.hpp>
#include <utility>

class Teleop : public rclcpp::Node
{
    static constexpr const char* TOPIC_JOY_ARM = "/base/joy/arm";
    static constexpr const char* TOPIC_ARM_STATUS = "/rover/arm/joints_status";
    static constexpr const char* TOPIC_ARM_CMD = "/rover/arm/joints_cmd";
    static constexpr std::chrono::milliseconds TELEOP_DEADLINE = std::chrono::milliseconds(200);
    static constexpr std::chrono::milliseconds TELEOP_LEASE_DURATION = std::chrono::milliseconds(300);

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
    bool _deadlineWarningActive = false;

  public:
    Teleop():
        rclcpp::Node("teleop_node"),
        _joyManager(),
        _jointController(_joyManager),
        _cartesianController(_joyManager)
    {
        rclcpp::QoS teleopQos(rclcpp::KeepLast(1));
        teleopQos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        teleopQos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        teleopQos.deadline(TELEOP_DEADLINE);
        teleopQos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
        teleopQos.liveliness_lease_duration(TELEOP_LEASE_DURATION);

        rclcpp::SubscriptionOptions teleopSubOptions;
        teleopSubOptions.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo& info_)
        {
            if (info_.total_count_change > 0 && !_deadlineWarningActive)
            {
                _deadlineWarningActive = true;
                RCLCPP_WARN(this->get_logger(),
                            "Teleop deadline missed: total=%d change=%d",
                            info_.total_count,
                            info_.total_count_change);

                rover_msgs::msg::ArmMsg armMsg;
                armMsg.target_speed = RobotController::getNullJointCommands();
                _pubArmCmd->publish(armMsg);
            }
        };

        teleopSubOptions.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessChangedInfo& info_)
        {
            if (info_.not_alive_count_change > 0)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Teleop liveliness lost: alive=%d not_alive=%d",
                            info_.alive_count,
                            info_.not_alive_count);
                rover_msgs::msg::ArmMsg armMsg;
                armMsg.target_speed = RobotController::getNullJointCommands();
                _pubArmCmd->publish(armMsg);
            }
        };

        _subJoyArm = this->create_subscription<rover_msgs::msg::Joy>(
            TOPIC_JOY_ARM,
            teleopQos,
            [this](const rover_msgs::msg::Joy& joyMsg_)
            {
                this->joy_CB(joyMsg_);
            },
            teleopSubOptions);
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
        _deadlineWarningActive = false;

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
            armMsg.target_speed = _jointController.getJointCmdFromInput(joyArray);
        }

        // CARTESIAN CONTROL
        else if (_controlMode == eControlMode::CARTESIAN)
        {
            _cartesianController.setJointPositions(_jointPositions);
            armMsg.target_speed = _cartesianController.getJointCmdFromInput(joyArray);

            // if (_joyManager.isTriggered(KEYBINDINGS::CARTESIAN::RECORD))
            // {
            //     if (_cartesianController.getRecordedPoints() == TO_UNDERLYING(CartesianController::eCartesianCoord::eLAST))
            //     {
            //         RCLCPP_WARN(this->get_logger(), "No more points can be recorded. Create a plan or clear all points");
            //     }
            //     else
            //     {
            //         _cartesianController.addPoint(_cartesianController.getEndEffectorPose(_jointPositions));
            //         RCLCPP_INFO(this->get_logger(), "Point has been added to array");
            //     }
            // }

            // if (_joyManager.isTriggered(KEYBINDINGS::CARTESIAN::CREATE_PLAN))
            // {
            //     if (_cartesianController.getRecordedPoints() != TO_UNDERLYING(CartesianController::eCartesianCoord::eLAST))
            //     {
            //         RCLCPP_WARN(this->get_logger(), "Cannot create plan since not enough points have been gathered");
            //     }
            //     else if (_cartesianController.applyPlan())
            //     {
            //         RCLCPP_INFO(this->get_logger(), "Applying plan");
            //     }
            //     else
            //     {
            //         RCLCPP_INFO(this->get_logger(), "Unapplying");
            //     }
            // }
        }

        _pubArmCmd->publish(armMsg);
    }

    void position_CB(const rover_msgs::msg::ArmMsg& armMsg_)
    {
        _jointPositions[std::to_underlying(eJointIndex::J0)] = armMsg_.current_position[rover_msgs::msg::ArmMsg::J0];
        _jointPositions[std::to_underlying(eJointIndex::J1)] = armMsg_.current_position[rover_msgs::msg::ArmMsg::J1];
        _jointPositions[std::to_underlying(eJointIndex::J2)] = armMsg_.current_position[rover_msgs::msg::ArmMsg::J2];
        _jointPositions[std::to_underlying(eJointIndex::GRIPPER_ROT)]
            = armMsg_.current_position[rover_msgs::msg::ArmMsg::GRIPPER_ROT];
        _jointPositions[std::to_underlying(eJointIndex::GRIPPER_TILT)]
            = armMsg_.current_position[rover_msgs::msg::ArmMsg::GRIPPER_TILT];
        _jointPositions[std::to_underlying(eJointIndex::GRIPPER_CLOSE)]
            = armMsg_.current_position[rover_msgs::msg::ArmMsg::GRIPPER_CLOSE];
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
}
