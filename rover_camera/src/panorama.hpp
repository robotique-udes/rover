#include <opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/srv/panorama.hpp>
#include <rover_msgs/msg/gps.hpp>
#include <iostream>
#include <vector>
#include <sys/stat.h>
#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>
#include <optional>

struct sCoordinate
{
    float latitude = 0.0F;
    float longitude = 0.0F;
};

class Panorama : public rclcpp::Node
{
  private:
    static constexpr float CROP_PERCENT = 0.10f;
    static constexpr uint8_t MAX_INVALID_FRAMES = 10U;
    static constexpr const char* PANORAMA_SERVICE_NAME = "/rover/video/panorama";
    static constexpr const char* TOPIC_GPS_NAME = "/rover/gps/position";
    static constexpr const char* PATH_FOR_PANORAMA = "/panorama";
    static constexpr const char* PIPELINE
        = " latency=0 drop=true ! decodebin ! videorate max-rate=2 ! videoconvert ! queue max-size-buffers=1 ! appsink";

  public:
    Panorama();

  private:
    void handlePanoramaRequest(const std::shared_ptr<rover_msgs::srv::Panorama::Request> request_,
                               std::shared_ptr<rover_msgs::srv::Panorama::Response> response_);
    bool validateRequest(const std::shared_ptr<rover_msgs::srv::Panorama::Request> request_,
                         std::shared_ptr<rover_msgs::srv::Panorama::Response> response_);
    bool captureFrames(const std::shared_ptr<rover_msgs::srv::Panorama::Request> request_,
                       std::shared_ptr<rover_msgs::srv::Panorama::Response> response_,
                       std::vector<cv::Mat>& frames_);
    void annotatePanorama(cv::Mat& pano, const std::string& name_);
    bool prepareOutputPath(const std::shared_ptr<rover_msgs::srv::Panorama::Request> request_,
                           std::shared_ptr<rover_msgs::srv::Panorama::Response> response_,
                           std::string& filename_);
    bool savePanorama(std::shared_ptr<rover_msgs::srv::Panorama::Response> response_,
                      const std::string& filename_,
                      const cv::Mat& pano_);
    cv::Mat warpCorrection(const cv::Mat& pano_);

    std::optional<std::string> getFolderPath(const std::string& basePath_);
    cv::Mat stitching(std::vector<cv::Mat>& frames_);
    void SetGpsPosition(const rover_msgs::msg::Gps& gpsMessage_);

    rclcpp::Service<rover_msgs::srv::Panorama>::SharedPtr srv_panorama;
    rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr sub_gps;
    sCoordinate _sCoordoneesGps;
};