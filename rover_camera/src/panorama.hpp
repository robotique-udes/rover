#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/srv/panorama.hpp>
#include <rover_msgs/msg/gps.hpp>
#include <rover_msgs/msg/camera_control.hpp>
#include <rover_msgs/msg/camera_config.hpp>
#include <vector>
#include <optional>

#define TEXT_COLOR cv::Scalar(34, 139, 34)

struct sCoordinate
{
    float latitude = 0.0F;
    float longitude = 0.0F;
};

class Panorama : public rclcpp::Node
{
  private:
    static constexpr float MAX_ROTATION_SPEED_PANORAMA = 15.0F;  // deg/s
    static constexpr float MAX_ROTATION_SPEED_GUI = 10.0F;       // rad/s
    static constexpr float MAX_PAN_ANGLE = 360.0F;               // deg
    static constexpr float MIDDLE_PAN_ANGLE = 180.0F;            // degs
    static constexpr float POSITION_TOLERANCE = 0.05F;           // rad
    static constexpr uint16_t STITCH_TIMEOUT_MS = 2'000U;
    static constexpr uint16_t ANGLE_WAIT_TIMEOUT_MS = 2'500U;
    static constexpr uint8_t PUBLISHER_CMD_PERIOD_MS = 50U;
    static constexpr uint8_t PUBLISHER_POWER_PERIOD_MS = 200U;
    static constexpr float CROP_PERCENT = 0.10F;
    static constexpr uint8_t MAX_INVALID_FRAMES = 10U;
    static constexpr double FONT_SCALE = 0.7;
    static constexpr int TEXT_THICKNESS = 3;
    static constexpr const char* TOPIC_CAMERA_PTZ_CMD_PANORAMA = "/rover/camera/PTZ_cmd/panorama";
    static constexpr const char* TOPIC_CAMERA_CONFIG_PANORAM = "/rover/camera/PTZ_config/panorama";
    static constexpr const char* TOPIC_CAMERA_POWER_PANORAMA = "/rover/camera/power_cmd/panorama";
    static constexpr const char* TOPIC_CAMERA_PTZ_STATUS = "/rover/camera/PTZ_status";
    static constexpr const char* PANORAMA_SERVICE_NAME = "/rover/video/panorama";
    static constexpr const char* TOPIC_GPS_NAME = "/rover/gps/position";
    static constexpr const char* PATH_FOR_PANORAMA = "/panorama";
    static constexpr const char* PANORAMA_FILE_NAME = "panorama_";
    static constexpr const char* PIPELINE
        = " latency=0 drop=true ! decodebin ! videorate max-rate=2 ! videoconvert ! queue max-size-buffers=1 ! appsink";

  public:
    Panorama();

  private:
    void CB_srvPanorama(const rover_msgs::srv::Panorama::Request& request_, rover_msgs::srv::Panorama::Response& response_);
    void handlePanoramaRequest(const rover_msgs::srv::Panorama::Request& request_,
                               rover_msgs::srv::Panorama::Response& response_,
                               uint8_t idCam_);
    void enableCameraPower(uint8_t id_);
    void disableCameraPower(uint8_t id_);
    bool validateRequest(const rover_msgs::srv::Panorama::Request& request_, rover_msgs::srv::Panorama::Response& response_);
    bool captureFrames(const rover_msgs::srv::Panorama::Request& request_,
                       rover_msgs::srv::Panorama::Response& response_,
                       std::vector<cv::Mat>& frames_);
    void annotatePanorama(cv::Mat& pano_, const std::string& name_);
    bool prepareOutputPath(const rover_msgs::srv::Panorama::Request& request_,
                           rover_msgs::srv::Panorama::Response& response_,
                           std::string& filename_);
    bool savePanorama(rover_msgs::srv::Panorama::Response& response_, const std::string& filename_, const cv::Mat& pano_);
    std::optional<cv::Mat> warpCorrection(const cv::Mat& pano_);
    void rotateCamera(uint16_t duration_, uint8_t idCam_);
    void waitForAngle(uint8_t idCam_, float angle_);
    std::optional<uint8_t> getIdCam(const std::string& camURL_);
    std::optional<std::string> getFolderPath(const std::string& basePath_);
    std::optional<cv::Mat> stitchFrames(std::vector<cv::Mat>& frames_);
    void setGpsPosition(const rover_msgs::msg::Gps& gpsMessage_);

    /**
     * @brief send the config PTZ msg using publisher
     *
     * @param idCam_ The id of the target camera
     * @param rotationSpeed_ rotation speed in rad/s
     */
    void configPtz(uint8_t idCam_, float rotationSpeed_);

    rclcpp::Service<rover_msgs::srv::Panorama>::SharedPtr _srv_panorama;
    rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr _sub_gps;
    rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _pub_cameraCmd;
    rclcpp::Publisher<rover_msgs::msg::CameraConfig>::SharedPtr _pub_cameraConfig;
    rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _pub_cameraPower;
    rclcpp::TimerBase::SharedPtr _timer_ptzCmd;
    rclcpp::TimerBase::SharedPtr _timer_powerCmd;
    sCoordinate _sGpsCoordinates;
};