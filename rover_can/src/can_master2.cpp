// ROS
#include <rclcpp/rclcpp.hpp>

#include "rover_can2/constant.hpp"
#include "rover_can2/drivers/driver_linux.hpp"
#include "rover_can2/can_msg.hpp"
#include "rover_lib2/helpers/log.hpp"

DEFINE_LOG_NODE(CanMaster, Logger::eNodeState::ON);

class CanMaster2 : public rclcpp::Node
{
  public:
    CanMaster2():
        Node("can_master2")
    {
        // Timer to show activity (optional)
        canDriver_.__init();

        // Prepare angle value to send
        float angle = 69.0f;
        std::array<uint8_t, 8> data = {};
        std::memcpy(data.data() + 2, &angle, sizeof(float)); // Store at index 2 (after MSG_ID and MSG_CONTENT_ID)

        // Set up message
        canMsg_ = RoverCan2::CanMsg(RoverCan2::Constant::eDeviceId::CAMERA_ROVER_FRONT, data.data(), 6);
        canMsg_.setMsgID(RoverCan2::Constant::eMsgId::CAM_POSITION_CMD);
        canMsg_.setMsgContentID(0); // Set if needed

        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&CanMaster2::timerCallback, this));
    }

  private:
    void timerCallback()
    {
        canDriver_._sendMsg(canMsg_);
    }

    RoverCan2::CanMsg canMsg_;
    rclcpp::TimerBase::SharedPtr timer_;
    RoverCan2::Drivers::DriverLinux canDriver_;
};

int main(int argc, char* argv[])
{
    LOG_INFO(Logger::Nodes::CanMaster, "TimerCallback");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanMaster2>());
    rclcpp::shutdown();

    return 0;
}