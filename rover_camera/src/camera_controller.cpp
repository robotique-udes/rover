#include "camera_controller.hpp"

#include <rover_lib2/helpers/constants.hpp>
#include <rover_lib2/helpers/ip_pinging.hpp>
#include <rover_lib2/helpers/rtsp_stream.hpp>

CameraController::CameraController():
    Node("camera_controller")
{
    _sub_camStatus
        = this->create_subscription<rover_msgs::msg::CameraControl>(TOPIC_CAMERA_STATUS,
                                                                    QOS_DEFAULT,
                                                                    [this](const rover_msgs::msg::CameraControl& camStatusMsg_)
                                                                    {
                                                                        this->CB_camStatus(camStatusMsg_);
                                                                    });
    _pub_camStatus = this->create_publisher<rover_msgs::msg::CameraControl>(TOPIC_CAMERA_STATUS, QOS_DEFAULT);
}

void CameraController::camStatusPub(void)
{
    rover_msgs::msg::CameraControl msg;
    msg.id_cam = _camID;
    msg.status = _camStatus;
    msg.pitch = _camPitch;
    msg.yaw = _camYaw;
}

void CameraController::CB_camAngle(const rover_msgs::msg::CameraControl& camAngleMsg_)
{
    
}

void CameraController::CB_camStatus(const rover_msgs::msg::CameraControl& camStatusMsg_)
{
    this->_camID = camStatusMsg_.id_cam;

    switch (this->_camID)
    {
        case CAM_MAIN:
            this->_camIP = "192.168.144.30";
            break;
        case CAM_ANTENNA:
            this->_camIP = "192.168.144.31";
            break;
        case CAM_FRONT_SIDE:
            this->_camIP = "192.168.144.32";
            break;
        case CAM_ARM_TOP:
            this->_camIP = "192.168.144.35";
            break;
        case CAM_ARM_SIDE:
            this->_camIP = "192.168.144.36";
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Invalid Cam ID");
            this->_camStatus = STATUS_ERROR;
            return;
    }

    if (!RoverLib2::isIPReachable(_camIP, 554, 500u))  // This is a placeholder
    {
        this->_camStatus = STATUS_NO_PING;
        return;
    }

    if (!RoverLib2::hasRTSPStream(_camIP, 500u))
    {
        this->_camStatus = STATUS_NO_STREAM;
        return;
    }

    this->camStatusPub();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CameraController>());

    rclcpp::shutdown();
    return 0;
}
