#include "camera_controller.hpp"

#include "rover_lib2/helpers/constants.hpp"
#include "rover_lib2/helpers/rtsp_stream.hpp"
#include "rover_lib2/helpers/ip_pinging.hpp"

CameraController::CameraController():
    Node("camera_controller")
{
    _sub_camPosControl = create_subscription<rover_msgs::msg::CameraControl>(TOPIC_CAMERA_POSITION,
                                                                             QOS_DEFAULT,
                                                                             [this](const rover_msgs::msg::CameraControl& msg_)
                                                                             {
                                                                                 this->CB_cameraPosControl(msg_);
                                                                             });

    _sub_camPower = create_subscription<rover_msgs::msg::CameraControl>(TOPIC_CAMERA_POWER_STATUS,
                                                                           QOS_DEFAULT,
                                                                           [this](const rover_msgs::msg::CameraControl& msg_)
                                                                           {
                                                                               this->CB_cameraPower(msg_);
                                                                           });

    _pub_camStatus = create_publisher<rover_msgs::msg::CameraControl>(TOPIC_CAMERA_STATUS, 1);

    _srv_cameraPower = this->create_service<rover_msgs::srv::CameraPower>(
        SERVICE_CAMERA_POWER,
        [this](const std::shared_ptr<rover_msgs::srv::CameraPower::Request> request_,
               std::shared_ptr<rover_msgs::srv::CameraPower::Response> response_)
        {
            this->CB_setCameraPower(request_, response_);
        });

    

    _timer_pub = create_wall_timer(std::chrono::milliseconds(PUBLISHER_PERIOD_MS),
                                   [this]()
                                   {
                                       this->publishAllCameraStatuses();
                                   });
}

CameraController::eCameraStatus CameraController::checkCameraStatus(eCameraID camID_)
{
    std::string cameraKey;
    switch (TO_UNDERLYING(camID_))
    {
        case rover_msgs::msg::CameraControl::ID_CAM_MAIN:
            cameraKey = "Main";
            break;
        case rover_msgs::msg::CameraControl::ID_CAM_ANTENNA:
            cameraKey = "Antenna";
            break;
        case rover_msgs::msg::CameraControl::ID_CAM_FRONT_SIDE:
            cameraKey = "Front-Side";
            break;
        case rover_msgs::msg::CameraControl::ID_CAM_ARM_TOP:
            cameraKey = "Arm-Top";
            break;
        case rover_msgs::msg::CameraControl::ID_CAM_ARM_SIDE:
            cameraKey = "Arm-Side";
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "checkCameraStatus(): invalid camera ID %u", TO_UNDERLYING(camID_));
            return eCameraStatus::STATUS_ERROR;
    }

    if (Constants::CameraInfo::CAMERA_URL_MAP.find(cameraKey) == Constants::CameraInfo::CAMERA_URL_MAP.end())
    {
        RCLCPP_ERROR(this->get_logger(), "checkCameraStatus(): no URL mapped for camera \"%s\"", cameraKey.c_str());
        return eCameraStatus::STATUS_ERROR;
    }

    std::string cameraURL = Constants::CameraInfo::CAMERA_URL_MAP.at(cameraKey);

    if (!RoverLib2::isIPReachable(cameraURL))
    {
        return eCameraStatus::STATUS_NO_PING;
    }

    if (!RoverLib2::hasRTSPStream(cameraURL, 500u))
    {
        return eCameraStatus::STATUS_NO_STREAM;
    }

    return eCameraStatus::STATUS_OK;
}

void CameraController::CB_cameraPower(const rover_msgs::msg::CameraControl& msg_)
{
    uint8_t camID = msg_.id_cam;
    if (camID >= TO_UNDERLYING(eCameraID::eLAST))
    {
        RCLCPP_WARN(this->get_logger(), "Power service: invalid ID %u", camID);
        return;
    }

    _powerOn[camID] = msg_.power_on;
}

void CameraController::CB_setCameraPower(const std::shared_ptr<rover_msgs::srv::CameraPower::Request> request_,
                                         std::shared_ptr<rover_msgs::srv::CameraPower::Response> response_)
{
    uint8_t id = request_->id_cam;
    if (id >= TO_UNDERLYING(eCameraID::eLAST))
    {
        response_->success = false;
        response_->message = "Invalid camera ID";
        RCLCPP_WARN(this->get_logger(), "Power service: invalid ID %u", id);
        return;
    }

    _powerOn[id] = request_->power_on;
    response_->success = true;
    response_->message = request_->power_on ? "Powered ON" : "Powered OFF";
}

void CameraController::CB_cameraPosControl(const rover_msgs::msg::CameraControl& msg_)
{
    uint8_t camID = msg_.id_cam;
    _camYaw[camID] = msg_.yaw;
    _camPitch[camID] = msg_.pitch;

    eCameraID eCamId;

    switch (camID)
    {
        case TO_UNDERLYING(eCameraID::CAMERA_MAIN):
            eCamId = eCameraID::CAMERA_MAIN;
            break;
        case TO_UNDERLYING(eCameraID::CAMERA_ANTENNA):
            eCamId = eCameraID::CAMERA_ANTENNA;
            break;
        case TO_UNDERLYING(eCameraID::CAMERA_FRONT_SIDE):
            eCamId = eCameraID::CAMERA_FRONT_SIDE;
            break;
        case TO_UNDERLYING(eCameraID::CAMERA_ARM_TOP):
            eCamId = eCameraID::CAMERA_ARM_TOP;
            break;
        case TO_UNDERLYING(eCameraID::CAMERA_ARM_SIDE):
            eCamId = eCameraID::CAMERA_ARM_SIDE;
            break;
        default:
            break;
    }

    publishCameraStatus(eCamId);
}

void CameraController::publishAllCameraStatuses()
{
    for (size_t i = 0; i < TO_UNDERLYING(eCameraID::eLAST); ++i)
    {
        // RCLCPP_INFO(this->get_logger(), "Publishing camera status for ID %d", i);
        publishCameraStatus(static_cast<eCameraID>(i));
    }
}

void CameraController::publishCameraStatus(eCameraID id)
{
    rover_msgs::msg::CameraControl msg;
    msg.id_cam = TO_UNDERLYING(checkCameraStatus(id));
    msg.yaw = _camYaw[TO_UNDERLYING(checkCameraStatus(id))];
    msg.pitch = _camPitch[TO_UNDERLYING(checkCameraStatus(id))];
    msg.status = TO_UNDERLYING(checkCameraStatus(id));
    _pub_camStatus->publish(msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraController>());
    rclcpp::shutdown();
    return 0;
}
