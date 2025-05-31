#ifndef __CAMERA__NODE__HPP__
#define __CAMERA__NODE__HPP__

#include "video_recording.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/gps.hpp>
#include <rover_msgs/srv/camera_control.hpp>
#include <rover_msgs/msg/camera_list.hpp>

#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>

#include <sys/stat.h>
#include <cstdlib>
#include <optional>

struct sScreenshotResult
{
    bool success;
    std::string msg;
};

class CameraNode : public rclcpp::Node
{
    static constexpr uint64_t PUBLISHER_PERIOD_MS = 200UL;
    static constexpr const char* SERVICE_MEDIA_SERVER_NAME = "/rover/cameras/media_server_control";
    static constexpr const char* TOPIC_MEDIA_SERVER_NAME = "/rover/camera/recordings_info";
    static constexpr const char* TOPIC_GPS_NAME = "/rover/gps/position";

    enum class eFileFormatNameTypes : size_t
    {
        SCREENSHOT,
        VIDEO,
    };

  public:
    CameraNode();
    ~CameraNode() = default;

  private:
    void controlIPCam(const rover_msgs::srv::CameraControl::Request& request_,
                      rover_msgs::srv::CameraControl::Response& response_);
    void takeScreenshot(const rover_msgs::srv::CameraControl::Request& request_,
                        rover_msgs::srv::CameraControl::Response& response_);
    void startRecordingLogic(const rover_msgs::srv::CameraControl::Request& request_,
                             rover_msgs::srv::CameraControl::Response& response_);
    void stopRecordingLogic(const rover_msgs::srv::CameraControl::Request& request_,
                            rover_msgs::srv::CameraControl::Response& response_);
    void CB_url_publisher(void);

    std::string getFileName(const std::string& capture_name_, std::string camURL_, eFileFormatNameTypes fileType_);
    std::optional<std::string> getFolderPath(const std::string& basePath_, eFileFormatNameTypes fileType_);
    void callbackPosition(const rover_msgs::msg::Gps& gps_message_);
    sScreenshotResult getScreenshot(std::string screenshotFolderPath_, std::string filename_, std::string cameraURL_);

    bool newRecording(std::string videoFolderPath_, std::string filename_, std::string cameraURL_);
    bool stopRecording(std::string cameraURL_);
    bool startWatchDog(void);
    void videoWatchDogFunction(void);
    void requestShutdown(std::string camURL_);

    rclcpp::Service<rover_msgs::srv::CameraControl>::SharedPtr _srv_control;
    rclcpp::Publisher<rover_msgs::msg::CameraList>::SharedPtr _pub_urls;
    rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr _sub_position;
    rclcpp::TimerBase::SharedPtr _timer_pub;

    float _lastLatitude = 0.0;
    float _lastLongitude = 0.0;

    std::atomic<bool> _watchDogStop{false};
    std::mutex _recordingMapMutex;
    std::thread _videoThread;
    std::condition_variable _recordingCv;
    std::unordered_map<std::string, Recording> _recordingMap;
    std::unordered_set<std::string> _recordingShutdownRequestSet;
};

#endif
