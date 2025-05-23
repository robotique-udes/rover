#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/camera_control.hpp>

class CameraController : public rclcpp::Node
{
    static constexpr uint64_t PUBLISHER_PERIOD_MS = 200UL;
    static constexpr const char* TOPIC_CAMERA_POSITION = "/rover/cameras/pos_control";
    static constexpr const char* TOPIC_CAMERA_STATUS = "/rover/camera/status";

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

    CameraController();

  private:
    void CB_cameraPosControl(const rover_msgs::msg::CameraControl& msg);
    void publishCameraStatus(eCameraID id);
    void publishAllCameraStatuses();

    eCameraStatus checkCameraStatus(eCameraID id);

    rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr _sub_camPosControl;
    rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _pub_camStatus;
    rclcpp::TimerBase::SharedPtr _timer_pub;

    std::array<double, static_cast<size_t>(eCameraID::eLAST)> _camYaw{};
    std::array<double, static_cast<size_t>(eCameraID::eLAST)> _camPitch{};
};
