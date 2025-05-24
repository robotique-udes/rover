#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/camera_control.hpp>
#include <rover_msgs/srv/camera_power.hpp>
#include "rover_lib2/helpers/macros.hpp"

#include <optional>

class CameraController : public rclcpp::Node
{
    static constexpr uint64_t PUBLISHER_PERIOD_MS = 300UL;
    static constexpr const char* TOPIC_CAMERA_POSITION = "/rover/cameras/pos_control";
    static constexpr const char* TOPIC_CAMERA_POWER_STATUS = "/rover/cameras/power_status";
    static constexpr const char* TOPIC_CAMERA_STATUS = "/rover/cameras/status";
    static constexpr const char* SERVICE_CAMERA_POWER = "/rover/cameras/power_control";

  public:
    enum class eCameraID
    {
        CAMERA_MAIN = 0,
        CAMERA_ANTENNA = 1,
        CAMERA_FRONT_SIDE = 2,
        CAMERA_ARM_TOP = 3,
        CAMERA_ARM_SIDE = 4,
        eLAST
    };

    enum class eCameraStatus
    {
        STATUS_OK = 0,
        STATUS_NO_PING = 1,
        STATUS_NO_STREAM = 2,
        STATUS_ERROR = 3,
    };

    struct sRtspUrl
    {
        std::string host;
        uint16_t port;
        std::string path;
    };

    CameraController();

  private:
    void CB_cameraPosControl(const rover_msgs::msg::CameraControl& msg_);
    void CB_cameraPower(const rover_msgs::msg::CameraControl& msg_);
    void publishCameraStatus(eCameraID id_);
    void publishAllCameraStatuses(void);
    void CB_setCameraPower(const std::shared_ptr<rover_msgs::srv::CameraPower::Request> request_,
                           std::shared_ptr<rover_msgs::srv::CameraPower::Response> response_);
    eCameraStatus checkCameraStatus(eCameraID id_);

    rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr _sub_camPosControl;
    rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr _sub_camPower;
    rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _pub_camStatus;
    rclcpp::Service<rover_msgs::srv::CameraPower>::SharedPtr _srv_cameraPower;
    rclcpp::TimerBase::SharedPtr _timer_pub;

    std::array<float, TO_UNDERLYING(eCameraID::eLAST)> _camYaw{};
    std::array<float, TO_UNDERLYING(eCameraID::eLAST)> _camPitch{};
    std::array<bool, TO_UNDERLYING(eCameraID::eLAST)> _powerOn{};
};
