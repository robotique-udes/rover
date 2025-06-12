#include "rover_lib2/helpers/cameraInterface.hpp"
#include <chrono>
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/camera_control.hpp>
#include <rover_lib2/helpers/constants.hpp>
#include <rover_msgs/msg/detail/camera_control__struct.hpp>

class CameraInterfaceTest : public rclcpp::Node
{
    static constexpr const char* TOPIC_SEND_COMMAND = "rover/camera/command";
    static constexpr const char* TOPIC_RECEIVE_STATUS = "rover/camera/status";

  public:
    CameraInterfaceTest():
        Node("camera_interface_test")
    {   
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this](void)
        {
            CB_cameraInterfaceTest();
        });

    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<CameraInterface> _testCameraInterface;

    void init()
    {
        _testCameraInterface = std::make_unique<CameraInterface>(shared_from_this(), TOPIC_SEND_COMMAND, TOPIC_RECEIVE_STATUS);

        rover_msgs::msg::CameraControl msg;
        msg.id_cam = 0;
        msg.yaw = 3;
        _testCameraInterface->setGoalMsg(msg);
    }

    void CB_cameraInterfaceTest()
    {
        if(_testCameraInterface)
        {
            rover_msgs::msg::CameraControl statusMsg = _testCameraInterface->getLastStatusMsg();
            uint8_t id = statusMsg.id_cam;
            float angle = statusMsg.yaw;
            RCLCPP_INFO(this->get_logger(), "id: %d , angle: %f", id, angle);
        }

    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<CameraInterfaceTest> node = std::make_shared<CameraInterfaceTest>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
