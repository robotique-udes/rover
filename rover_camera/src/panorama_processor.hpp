#ifndef PANORAMA_PROCESSOR_HPP
#define PANORAMA_PROCESSOR_HPP

#include "rover_lib2/helpers/constants.hpp"
#include "rover_lib2/helpers/cameraInterface.hpp"

#include <rover_msgs/msg/camera_control.hpp>
#include <rover_msgs/msg/camera_config.hpp>
#include <rover_msgs/srv/panorama.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <optional>
#include <atomic>

#define TEXT_COLOR cv::Scalar(34, 139, 34)

struct sCoordinate
{
    float latitude = 0.0F;
    float longitude = 0.0F;
};

class PanoramaProcessor
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
    static constexpr const char* TOPIC_CAMERA_PTZ_STATUS = "/rover/camera/PTZ_status";
    static constexpr const char* PATH_FOR_PANORAMA = "/panorama";
    static constexpr const char* PANORAMA_FILE_NAME = "panorama_";
    static constexpr const char* PIPELINE
        = " latency=0 drop=true ! decodebin ! videorate max-rate=2 ! videoconvert ! queue max-size-buffers=1 ! appsink";

  public:
    PanoramaProcessor(Constants::CameraInfo::eCamNames id_, std::shared_ptr<CameraInterface> cameraInterface_);
    ~PanoramaProcessor();

    void execute(const rover_msgs::srv::Panorama::Request& request_,
                 rover_msgs::srv::Panorama::Response& response_,
                 sCoordinate coordinates_);
    bool isBusy(void);

  private:
    void handlePanoramaRequest(const rover_msgs::srv::Panorama::Request& request_,
                               rover_msgs::srv::Panorama::Response& response_,
                               Constants::CameraInfo::eCamNames id_, sCoordinate coordinates_);
    void enableCameraPower(Constants::CameraInfo::eCamNames id_);
    void disableCameraPower(Constants::CameraInfo::eCamNames id_);
    bool validateRequest(const rover_msgs::srv::Panorama::Request& request_, rover_msgs::srv::Panorama::Response& response_);
    bool captureFrames(const rover_msgs::srv::Panorama::Request& request_,
                       rover_msgs::srv::Panorama::Response& response_,
                       std::vector<cv::Mat>& frames_);
    void annotatePanorama(cv::Mat& pano_, const std::string& name_, sCoordinate coordinates_);
    bool prepareOutputPath(const rover_msgs::srv::Panorama::Request& request_,
                           rover_msgs::srv::Panorama::Response& response_,
                           std::string& filename_);
    bool savePanorama(rover_msgs::srv::Panorama::Response& response_, const std::string& filename_, const cv::Mat& pano_);
    std::optional<cv::Mat> warpCorrection(const cv::Mat& pano_);
    void rotateCamera(uint16_t duration_, Constants::CameraInfo::eCamNames id_);
    void waitForAngle(Constants::CameraInfo::eCamNames id_, float angle_);
    std::optional<std::string> getFolderPath(const std::string& basePath_);
    std::optional<cv::Mat> stitchFrames(std::vector<cv::Mat>& frames_);

    /**
     * @brief send the config PTZ msg using publisher
     *
     * @param idCam_ The id of the target camera
     * @param rotationSpeed_ rotation speed in rad/s
     */
    void configPtz(Constants::CameraInfo::eCamNames id_, float rotationSpeed_);

    std::atomic<bool> _busy{false};
    Constants::CameraInfo::eCamNames _id;
    std::shared_ptr<CameraInterface> _cameraInterface;
};

#endif  // defined PANORAMA_PROCESSOR_HPP