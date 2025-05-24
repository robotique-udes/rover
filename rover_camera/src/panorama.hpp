#include <opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/photo_panoramique.hpp>
#include <rover_msgs/srv/photo_panoramique.hpp>
#include <rover_msgs/msg/gps_position.hpp>
#include <iostream>
#include <vector>
#include <sys/stat.h>
#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>

struct emplacement
{
    float latitude;
    float longitude;
};

class PhotoPanoramique : public rclcpp::Node
{
  private:
  static constexpr uint8_t FPS = 30U;
  static constexpr float CROP_PERCENT = 0.10f;
  static constexpr size_t FRAMES_TO_SKIP = 5U;
  static constexpr uint8_t MAX_INVALID_FRAMES = 10U;

  public:
    PhotoPanoramique();

  private:
    // attribut pour gérer les coordonées gps
    emplacement sCoordoneesGps;

    // fonction pour enlever le warping
    cv::Mat warpCorrection(const cv::Mat& pano);

    std::string getCurrentTime(void);
    bool createFolder(const std::string& path_);
    bool folderExists(const std::string& path_);

    // fonction pour le stitching de la photo
    cv::Mat stitching(std::vector<cv::Mat>& imagesCam);

    // fonction pour aller chercher la position GPS
    void PositionGPS(const rover_msgs::msg::GpsPosition& gpsMessage_);

    // section necessitees ROS
    rclcpp::Service<rover_msgs::srv::PhotoPanoramique>::SharedPtr srv_panorama;
    rclcpp::Subscription<rover_msgs::msg::GpsPosition>::SharedPtr sub_position;

    rover_msgs::msg::PhotoPanoramique _msgPanorama;
    void CB_srv(const std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Request> request_,
                std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Response> response_);
};