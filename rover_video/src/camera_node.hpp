#ifndef __CAMERA__NODE__HPP__
#define __CAMERA__NODE__HPP__

#include "video_recording.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rovus_lib/macros.h"
#include "rover_msgs/msg/gps_position.hpp"
#include "rover_msgs/srv/camera_control.hpp"

#include <sys/stat.h>
#include <cstdlib>

class CameraNode : public rclcpp::Node
{
    enum class eFileFormatNameTypes : size_t
    {
        SCREENSHOT,
        VIDEO,
    };

  public:
    CameraNode(void);
    ~CameraNode() = default;

  private:
    void controlIPCam(const rover_msgs::srv::CameraControl::Request& request_,
                      rover_msgs::srv::CameraControl::Response& response_);
    std::string getCurrentTime(void);
    std::string getFileName(const std::string& capture_name_, std::string camURL_, eFileFormatNameTypes state_);
    std::string getCamID(std::string cameraURL);
    void callbackPosition(const rover_msgs::msg::GpsPosition& gps_message_);
    const std::string getFolderPath(eFileFormatNameTypes state_);
    bool folderExists(const std::string& path_);
    bool createFolder(const std::string& path_);
    bool getScreenshot(std::string screenshotFolderPath_, std::string filename_, std::string cameraURL_);

    bool newRecording(std::string videoFolderPath_, std::string filename_, std::string cameraURL_);
    bool stopRecording(std::string cameraURL_);
    bool StartWatchDog(void);
    void VideoWatchDogFunction(void);
    void RequestShutdown(std::string camURL_);

    rclcpp::Service<rover_msgs::srv::CameraControl>::SharedPtr _srv_control;
    rclcpp::Subscription<rover_msgs::msg::GpsPosition>::SharedPtr _msg_position;

    float _last_latitude = 0.0;
    float _last_longitude = 0.0;

    std::atomic<bool> _watchDogStop{false};
    std::mutex _recordingMutex;
    std::thread _videoWatchDog;
    std::condition_variable _recordingCv;
    std::unordered_map<std::string, Recording> _RecordingMap;
    std::unordered_set<std::string> _RecordingShutdownRequestSet;
};

#endif