#ifndef __JOINT_CONTROLLER_HPP__
#define __JOINT_CONTROLLER_HPP__

#include "robot_controller.hpp"
#include "Eigen/Dense"
#include "rover_msgs/msg/arm_msg.hpp"

class JointController : public RobotController<static_cast<size_t>(JointController::eJointIndex::eLAST)>
{
  public:
    enum class eJointIndex : uint8_t
    {
        JL = rover_msgs::msg::ArmMsg::JL,
        J1 = rover_msgs::msg::ArmMsg::J1,
        J2 = rover_msgs::msg::ArmMsg::J2,
        GRIPPER_TILT = rover_msgs::msg::ArmMsg::GRIPPER_TILT,
        GRIPPER_ROT = rover_msgs::msg::ArmMsg::GRIPPER_ROT,
        GRIPPER_CLOSE = rover_msgs::msg::ArmMsg::GRIPPER_CLOSE,
        eLAST
    };

    JointController()
        : RobotController(RobotController::eControlMode::JOINT) {}

    void setCmd(std::array<size_t, static_cast<size_t>(eJointIndex::eLAST)> cmd) override
    {
        // Implement control logic for setting joint commands
        _cmd = cmd;
    }

    void setMode(eControlMode mode_) override
    {
        _controlMode = mode_;
    }

  private:
    std::array<size_t, static_cast<size_t>(eJointIndex::eLAST)> _cmd;
};

#endif
