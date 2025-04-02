#ifndef __CAMERA__NODE__HPP__
#define __CAMERA__NODE__HPP__

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/gps_position.hpp"
#include "rover_msgs/srv/camera_control.hpp"
#include "rovus_lib/macros.h"
#include "video_recording.hpp"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

#include <cstdlib>
#include <sys/stat.h>

enum class eFileFormatNameTypes : size_t
{
    SCREENSHOT,
    VIDEO,
};


class CameraNode : public rclcpp::Node
{
    rclcpp::Service<rover_msgs::srv::CameraControl>::SharedPtr _srv_control;
    rclcpp::Subscription<rover_msgs::msg::GpsPosition>::SharedPtr _msg_position;
    float last_latitude = 0.0;
    float last_longitude = 0.0;

    void controlIPCam(const rover_msgs::srv::CameraControl::Request& request, rover_msgs::srv::CameraControl::Response& response);
    std::string getCurrentTime();
    std::string getFileName(const std::string& capture_name, std::string camURL, eFileFormatNameTypes state);
    std::string getCamID(std::string cameraURL);
    void callbackPosition(const rover_msgs::msg::GpsPosition& gps_message);
    const std::string getFolderPath(eFileFormatNameTypes state);
    bool folderExists(const std::string& path);
    bool createFolder(const std::string& path);
    bool getScreenshot(std::string screenshotFolderPath, std::string filename, std::string cameraURL);

    bool newRecording(std::string videoFolderPath, std::string filename, std::string cameraURL);
    bool stopRecording(std::string cameraURL);
    bool StartWatchDog();
    void VideoWatchDogFunction();
    void RequestShutdown(std::string camURL);
    std::atomic<bool> watchDogStop{false};
    std::mutex recordingMutex;
    std::thread videoWatchDog;
    std::condition_variable recordingCv;

    std::unordered_map<std::string, Recording> RecordingMap;
    std::unordered_set<std::string> RecordingShutdownRequestSet;

  public:
    CameraNode();
    ~CameraNode() {}
};

#endif