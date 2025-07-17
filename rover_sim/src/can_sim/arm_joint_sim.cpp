#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/arm_msg.hpp>
#include <rover_can2/rover_can2.hpp>
#include <rover_can2/msgs/arm_speed_cmd.hpp>
#include <rover_can2/msgs/arm_position_status.hpp>


class ArmJointSimulator : public rclcpp::Node
{
  private:
    static constexpr const char* ARM_CMD_TOPIC = "/rover/arm/joints_cmd";
    static constexpr uint32_t CAN_PUBLISH_PERIOD_MS = 50;

  public:
    ArmJointSimulator(uint8_t rosArmSpeedMsgId_):
        Node("arm_joint_simulator"), _rosArmSpeedMsgId(rosArmSpeedMsgId_)
    {
        _driver.init(); 
        _pubSpeed = this->create_publisher<rover_msgs::msg::ArmMsg>(ARM_CMD_TOPIC, 1);
        _timerPub = this->create_wall_timer(std::chrono::milliseconds(CAN_PUBLISH_PERIOD_MS),
                                            [this]()
                                            {
                                                this->CB_rosPub();
                                            });
    };

  private:
    void CB_rosPub(void) 
    {
        rover_msgs::msg::ArmMsg msg;
        msg.target_speed[_rosArmSpeedMsgId] = 1.0f;
        _pubSpeed->publish(msg);
    }

    rclcpp::Publisher<rover_msgs::msg::ArmMsg>::SharedPtr _pubSpeed;
    rclcpp::TimerBase::SharedPtr _timerPub;

    uint8_t _rosArmSpeedMsgId;

};
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmJointSimulator>(0));
    rclcpp::shutdown();
    return 0;
}