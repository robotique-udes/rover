#ifndef __CAMERA_CONTROLLER_HPP
#define __CAMERA_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/camera_control.hpp>

class CameraController : public rclcpp::Node
{
    enum CameraID : uint8_t
    {
        CAM_MAIN = 0,
        CAM_ANTENNA = 1,
        CAM_FRONT_SIDE = 2,
        CAM_ARM_TOP = 3,
        CAM_ARM_SIDE = 4,
        eLAST
    };

    enum CameraStatus : uint8_t
    {
        STATUS_NONE = 0,
        STATUS_OK = 1,
        STATUS_ERROR = 2,
        STATUS_NO_PING = 2,
        STATUS_NO_STREAM = 3,
        eLAST
    };

    static constexpr char* SERVICE_POWER_CONTROL = "/rover/cameras/power_control";
    static constexpr char* TOPIC_POWER_STATUS = "/rover/cameras/power_status";
    static constexpr char* TOPIC_CAMERA_STATUS = "/rover/cameras/status";
    static constexpr char* TOPIC_CAMERA_ANGLE = "/rover/cameras/angle";

  public:
    CameraController();
    ~CameraController() = default;

  private:
    void CB_camStatus(const rover_msgs::msg::CameraControl& camStatusMsg_);
    void CB_camAngle(const rover_msgs::msg::CameraControl& camAngleMsg_);
    void camStatusPub(void);

    rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr _sub_camStatus;
    rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr _sub_camAngle;
    rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _pub_camStatus;
    rclcpp::TimerBase::SharedPtr _timerPub;

    std::string _camIP;
    uint8_t _camID;
    uint8_t _camStatus;
    float _camPitch;
    float _camYaw;

};

#endif