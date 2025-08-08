#include "panorama_manager.hpp"

#include <sys/stat.h>
#include <rover_lib2/helpers/folders.hpp>
#include <rover_lib2/helpers/date.hpp>
#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>

PanoramaManager::PanoramaManager():
    Node("PanoramaManager"),
    _cameraInterface(std::make_shared<CameraInterface>(this,
                                                       TOPIC_CAMERA_PTZ_CMD_PANORAMA,
                                                       TOPIC_CAMERA_CONFIG_PANORAM,
                                                       TOPIC_CAMERA_POWER_PANORAMA))
{
    _srv_panorama = this->create_service<rover_msgs::srv::Panorama>(
        PANORAMA_SERVICE_NAME,
        [this](const std::shared_ptr<rover_msgs::srv::Panorama::Request> request_,
               std::shared_ptr<rover_msgs::srv::Panorama::Response> response_)
        {
            this->CB_srvPanorama(*request_, *response_);
        });

    _sub_gps = this->create_subscription<rover_msgs::msg::Gps>(TOPIC_GPS_NAME,
                                                               QOS_DEFAULT,
                                                               [this](const rover_msgs::msg::Gps& gpsMsg_)
                                                               {
                                                                   this->setGpsPosition(gpsMsg_);
                                                               });
}

void PanoramaManager::CB_srvPanorama(const rover_msgs::srv::Panorama::Request& request_,
                                     rover_msgs::srv::Panorama::Response& response_)
{
    std::optional<Constants::CameraInfo::eCamNames> idCam = Constants::CameraInfo::getIdFromURL(request_.camera_url);
    if (!idCam)
    {
        response_.success = false;
        response_.status = "Invalid camera URL";
        return;
    }

    _panoramaProcessors[std::to_underlying(idCam)].execute(request_, response_, _sGpsCoordinates);
}

void PanoramaManager::setGpsPosition(const rover_msgs::msg::Gps& gpsMsg_)
{
    _sGpsCoordinates.latitude = gpsMsg_.latitude;
    _sGpsCoordinates.longitude = gpsMsg_.longitude;
}

void PanoramaManager::initPanoramaProcessor(void)
{
    for (size_t id = 0; id < std::to_underlying(Constants::CameraInfo::eCamNames::eLast); ++id)
    {
        _panoramaProcessors[id] = PanoramaProcessor(static_cast<Constants::CameraInfo::eCamNames>(id));
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PanoramaManager>());
    rclcpp::shutdown();
    return 0;
}
