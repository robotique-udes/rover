#include "panorama_processor.hpp"

#include <array>
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/gps.hpp>

class PanoramaManager : public rclcpp::Node
{
  private:
    static constexpr const char* TOPIC_GPS_NAME = "/rover/gps/position";
    static constexpr const char* PANORAMA_SERVICE_NAME = "/rover/video/panorama";
    static constexpr const char* TOPIC_CAMERA_PTZ_CMD_PANORAMA = "/rover/camera/PTZ_cmd/panorama";
    static constexpr const char* TOPIC_CAMERA_CONFIG_PANORAM = "/rover/camera/PTZ_config/panorama";
    static constexpr const char* TOPIC_CAMERA_POWER_PANORAMA = "/rover/camera/power_cmd/panorama";

  public:
    PanoramaManager();
    void initPanoramaProcessor(void);
    void initCameraInterface(void);

  private:
    void CB_srvPanorama(const rover_msgs::srv::Panorama::Request& request_, rover_msgs::srv::Panorama::Response& response_);
    void setGpsPosition(const rover_msgs::msg::Gps& gpsMessage_);

    rclcpp::Service<rover_msgs::srv::Panorama>::SharedPtr _srv_panorama;
    rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr _sub_gps;
    sCoordinate _sGpsCoordinates;
    std::array<std::unique_ptr<PanoramaProcessor>, std::to_underlying(Constants::CameraInfo::eCamNames::eLast)>
        _panoramaProcessors;
    std::shared_ptr<CameraInterface> _cameraInterface;
};