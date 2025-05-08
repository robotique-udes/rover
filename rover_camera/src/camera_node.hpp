#ifndef __CAMERA__NODE__HPP__
#define __CAMERA__NODE__HPP__

#include "video_recording.hpp"

#include <rover_msgs/msg/gps_position.hpp>
#include <rover_msgs/srv/camera_control.hpp>

#include <sys/stat.h>
#include <cstdlib>

class CameraNode : public rclcpp::Node
{
    static constexpr const char* SERVICE_MEDIA_SERVER_NAME = "/rover/cameras/media_server_control";

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

    std::string getCurrentTime(void);
    std::string getFileName(const std::string& capture_name_, std::string camURL_, eFileFormatNameTypes fileType_);
    const std::string getFolderPath(eFileFormatNameTypes fileType_);
    void callbackPosition(const rover_msgs::msg::GpsPosition& gps_message_);
    bool folderExists(const std::string& path_);
    bool createFolder(const std::string& path_);
    bool getScreenshot(std::string screenshotFolderPath_, std::string filename_, std::string cameraURL_);

    bool newRecording(std::string videoFolderPath_, std::string filename_, std::string cameraURL_);
    bool stopRecording(std::string cameraURL_);
    bool startWatchDog(void);
    void videoWatchDogFunction(void);
    void requestShutdown(std::string camURL_);

    rclcpp::Service<rover_msgs::srv::CameraControl>::SharedPtr _srv_control;
    rclcpp::Subscription<rover_msgs::msg::GpsPosition>::SharedPtr _sub_position;

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
