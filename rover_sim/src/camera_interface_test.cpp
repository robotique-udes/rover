#include "rover_lib2/helpers/cameraInterface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/camera_control.hpp>
#include <rover_lib2/helpers/constants.hpp>

class CameraInterfaceTest : public rclcpp::Node
{
    static constexpr const char* TOPIC_SEND_PTZ_COMMAND_PANORAMA = "/rover/camera/PTZ_cmd/panorama";
    static constexpr const char* TOPIC_SEND_CONFIG_COMMAND_PANORAMA = "/rover/camera/PTZ_config/panorama";
    static constexpr const char* TOPIC_SEND_POWER_COMMAND_PANORAMA = "/rover/camera/power_cmd/panorama";

    static constexpr const char* TOPIC_SEND_PTZ_COMMAND_GUI = "/rover/camera/PTZ_cmd/GUI";
    static constexpr const char* TOPIC_SEND_CONFIG_COMMAND_GUI = "/rover/camera/PTZ_config/GUI";
    static constexpr const char* TOPIC_SEND_POWER_COMMAND_GUI = "/rover/camera/power_cmd/GUI";

    static constexpr const uint64_t GOAL_WATCHDOG_DELAY = 1000;

    static constexpr const size_t TEST_CAM_A_ID = 0;
    static constexpr const size_t TEST_CAM_B_ID = 3;

  public:
    CameraInterfaceTest():
        Node("camera_interface_test")
    {
        _timer_simulation = this->create_wall_timer(std::chrono::milliseconds(15000),
                                                    [this](void)
                                                    {
                                                        CB_cameraInterfaceTest();
                                                    });

        _timer_goalWatchdog = this->create_wall_timer(
            std::chrono::milliseconds(GOAL_WATCHDOG_DELAY),
            [this](void)
            {
                if (_testCameraInterfaceGUI->isGoalReached(TEST_CAM_B_ID))
                {
                    RCLCPP_INFO(rclcpp::get_logger("CAMERA_SIM"), "GOAL REACHED ON %ld ", TEST_CAM_B_ID);
                }
            });
    }

    rclcpp::TimerBase::SharedPtr _timer_simulation;
    rclcpp::TimerBase::SharedPtr _timer_goalWatchdog;
    std::unique_ptr<CameraInterface> _testCameraInterfacePanorama;
    std::unique_ptr<CameraInterface> _testCameraInterfaceGUI;

    size_t simulationSwippingIndex = 0;

    void init()
    {
        _testCameraInterfacePanorama = std::make_unique<CameraInterface>(shared_from_this(),
                                                                         TOPIC_SEND_PTZ_COMMAND_PANORAMA,
                                                                         TOPIC_SEND_CONFIG_COMMAND_PANORAMA,
                                                                         TOPIC_SEND_POWER_COMMAND_PANORAMA);

        _testCameraInterfaceGUI = std::make_unique<CameraInterface>(shared_from_this(),
                                                                    TOPIC_SEND_PTZ_COMMAND_GUI,
                                                                    TOPIC_SEND_CONFIG_COMMAND_GUI,
                                                                    TOPIC_SEND_POWER_COMMAND_GUI);

        rover_msgs::msg::CameraControl msg;
        size_t id = TEST_CAM_A_ID;
        msg.id_cam = id;
        msg.yaw = 1;
        _testCameraInterfaceGUI->setPTZCmd(msg, id);
    }

    void CB_cameraInterfaceTest()
    {
        if (_testCameraInterfaceGUI && _testCameraInterfacePanorama)
        {
            bool underControl = _testCameraInterfacePanorama->isCamUnderControl(TEST_CAM_B_ID);
            ;

            RCLCPP_INFO(rclcpp::get_logger("CAMERA_SIM"), "Panorama as now control based on feedback?: %d", underControl);

            if (simulationSwippingIndex > 1)
            {
                _testCameraInterfaceGUI->release(TEST_CAM_A_ID);
            }

            if (simulationSwippingIndex % 2)
            {
                rover_msgs::msg::CameraControl msg;
                size_t id = TEST_CAM_B_ID;
                msg.id_cam = id;
                msg.yaw = 4;
                _testCameraInterfacePanorama->setPTZCmd(msg, TEST_CAM_B_ID);
            }
            else
            {
                _testCameraInterfacePanorama->release(TEST_CAM_B_ID);
                rover_msgs::msg::CameraControl msg;
                size_t id = TEST_CAM_B_ID;
                msg.id_cam = id;
                msg.yaw = 2;
                _testCameraInterfaceGUI->setPTZCmd(msg, TEST_CAM_B_ID);
                ;
            }
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
