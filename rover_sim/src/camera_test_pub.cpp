#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/camera_control.hpp>

class CameraTestPub : public rclcpp::Node
{
    static constexpr size_t SIMULATED_CAM_ID = 3;
    static constexpr char* TOPIC_PTZ_STATUS = "/rover/camera/PTZ_status";
    static constexpr char* TOPIC_PTZ_CMD = "/rover/camera/PTZ_cmd/manager";

  public:
    CameraTestPub():
        Node("camera_test_pub")
    {
        _publisher = this->create_publisher<rover_msgs::msg::CameraControl>(TOPIC_PTZ_STATUS, 1);
        _subscriber
            = this->create_subscription<rover_msgs::msg::CameraControl>(TOPIC_PTZ_CMD,
                                                                        1,
                                                                        [this](const rover_msgs::msg::CameraControl& PTZcmd_)
                                                                        {
                                                                            this->CB_receivePTZcmd(PTZcmd_);
                                                                        });
    }

  private:
    void CB_receivePTZcmd(rover_msgs::msg::CameraControl PTZcmd_)
    {
        size_t id = PTZcmd_.id_cam;
        if (id != SIMULATED_CAM_ID)
        {
            return;
        }

        double targetYaw = PTZcmd_.yaw;

        if (targetYaw < currentYaw)
        {
            if (currentYaw - targetYaw < 0.15)
            {
                currentYaw -= 0.05;
            }
            else
            {
                currentYaw -= 0.1;
            }
        }
        else if (targetYaw > currentYaw)
        {
            if (targetYaw - currentYaw < 0.15)
            {
                currentYaw += 0.05;
            }
            else
            {
                currentYaw += 0.1;
            }
        }

        rover_msgs::msg::CameraControl statusMsg;
        statusMsg.id_cam = id;
        statusMsg.yaw = currentYaw;
        _publisher->publish(statusMsg);
    }

    rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _publisher;
    rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr _subscriber;

    double currentYaw = 0;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraTestPub>());
    rclcpp::shutdown();
    return 0;
}
