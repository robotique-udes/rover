#include "rover_lib2/helpers/cameraInterface.hpp"
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/camera_control.hpp>
#include <rover_lib2/helpers/constants.hpp>
#include <rover_msgs/msg/detail/camera_control__struct.hpp>

class CameraInterfaceTest : public rclcpp::Node
{
    static constexpr const char* TOPIC_SEND_COMMAND_PANORAMA = "rover/camera/PTZcmd/panorama";
    static constexpr const char* TOPIC_RECEIVE_STATUS_PANORAMA = "rover/camera/PTZstatus";
    static constexpr const char* TOPIC_SEND_COMMAND_GUI = "rover/camera/PTZcmd/GUI";

  public:
    CameraInterfaceTest():
        Node("camera_interface_test")
    {
        _timerPanorama = this->create_wall_timer(std::chrono::milliseconds(500),
                                                 [this](void)
                                                 {
                                                     CB_cameraInterfaceTest();
                                                 });

        _timerGUI = this->create_wall_timer(std::chrono::milliseconds(5000),
                                            [this](void)
                                            {
                                                CB_cameraInterfaceTestGUI();
                                            });
    }

    rclcpp::TimerBase::SharedPtr _timerPanorama;
    rclcpp::TimerBase::SharedPtr _timerGUI;
    std::unique_ptr<CameraInterface> _testCameraInterfacePanorama;

    size_t simulationSwippingIndex = 0;

    void init()
    {
        _testCameraInterfacePanorama
            = std::make_unique<CameraInterface>(shared_from_this(), TOPIC_SEND_COMMAND_PANORAMA, TOPIC_RECEIVE_STATUS_PANORAMA);

        rover_msgs::msg::CameraControl msg1;
        size_t id1 = 0;
        msg1.id_cam = id1;
        msg1.yaw = 3;
        _testCameraInterfacePanorama->setGoalMsg(msg1, id1);

        rover_msgs::msg::CameraControl msg2;
        size_t id2 = 3;
        msg2.id_cam = id2;
        msg2.yaw = 1;
        _testCameraInterfacePanorama->setGoalMsg(msg2, id2);
    }

    void CB_cameraInterfaceTest()
    {
        if (_testCameraInterfacePanorama)
        {
            rover_msgs::msg::CameraControl statusMsg1 = _testCameraInterfacePanorama->getLastStatusMsg(0);
            uint8_t id1 = statusMsg1.id_cam;
            float angle1 = statusMsg1.yaw;
            RCLCPP_INFO(this->get_logger(), "id: %d , angle: %f", id1, angle1);

            rover_msgs::msg::CameraControl statusMsg2 = _testCameraInterfacePanorama->getLastStatusMsg(3);
            uint8_t id2 = statusMsg2.id_cam;
            float angle2 = statusMsg2.yaw;
            RCLCPP_INFO(this->get_logger(), "id: %d , angle: %f", id2, angle2);
        }
    }

    void CB_cameraInterfaceTestGUI()
    {
        if (!(simulationSwippingIndex % 2))
        {
            rover_msgs::msg::CameraControl msg2;
            size_t id2 = 3;
            msg2.id_cam = id2;
            msg2.yaw = 22;
            _testCameraInterfacePanorama->setGoalMsg(msg2, id2);
        }
        else
        {

        }
        simulationSwippingIndex++;
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
