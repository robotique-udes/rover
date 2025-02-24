#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/srv/camera_control.hpp"
#include "rovus_lib/macros.h"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <cstdlib>
#include <sys/stat.h>

#define SCREENSHOT 1
#define VIDEO 2

/* This folder is used for the functions' declarations */

class CameraNode : public rclcpp::Node
{
  public:
  private:
    rclcpp::Service<rover_msgs::srv::CameraControl>::SharedPtr _srv_screenshot;
    rclcpp::Service<rover_msgs::srv::CameraControl>::SharedPtr _srv_recording;

    void controlIPCam(const std::shared_ptr<rover_msgs::srv::CameraControl::Request> request,
                      std::shared_ptr<rover_msgs::srv::CameraControl::Response> response);
    void recordingIPCam(const std::shared_ptr<rover_msgs::srv::CameraControl::Request> request,
                        std::shared_ptr<rover_msgs::srv::CameraControl::Response> response);
    std::string getCurrentTime();
    std::string getFileName(const std::string capture_name);
    const std::string getFolderPath(int state);
    bool folderExists(const std::string& path);
    bool createFolder(const std::string& path);
    bool getScreenshot(std::string screenshotFolderPath, std::string filename, std::string cameraURL);
    bool startRecording();
    bool stopRecording();

  public:
    CameraNode();
    ~CameraNode() {}
};

std::string CameraNode::getCurrentTime()
{
    auto now = std::chrono::system_clock::now();  // get system time

    auto now_time = std::chrono::system_clock::to_time_t(now);  // convert to real time

    std::tm tm_now = *std::localtime(&now_time);  // convert to calendar time

    std::stringstream current_time_output;
    current_time_output << std::put_time(&tm_now, "%FT%T");  // ISO 8601 format

    return current_time_output.str();
}

std::string CameraNode::getFileName(const std::string capture_name)
{
    std::string filename = capture_name.empty() ? getCurrentTime() + "_screenshot.png" : capture_name;
    return filename;
}

const std::string CameraNode::getFolderPath(int state)
{
    std::string folderPath;

    std::string dir = GET_PACKAGE_SOURCE_DIR("rover_video");  // finds the path to our package

    switch (state)
    {
        case SCREENSHOT:
            // Path necessary for the screenshots folder
            folderPath = std::string(dir) + "/src/screenshots";
            break;

        case VIDEO:
            // Path necessary for the recordings folder
            folderPath = std::string(dir) + "/src/recordings";
            break;
    }

    return folderPath;
}

// Verifies if screenshot folder already exists
bool CameraNode::folderExists(const std::string& path)
{
    struct stat info;
    return (stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR));  // I dont exactly understand this part
}

// Creating screenshot Folder if doesnt already exists
bool CameraNode::createFolder(const std::string& path)
{
    if (!folderExists(path))
    {
        if (mkdir(path.c_str(), 0777) == 0)
        {  // 0777 = Full permissions
            RCLCPP_INFO(LOGGER, "Succesfully created the folder.");
            return true;
        }
        else
        {
            RCLCPP_INFO(LOGGER, "Couldn't create the folder.");
            return false;
        }
    }

    RCLCPP_INFO(LOGGER, "Directory already exists: %s", path.c_str());
    return true;
}

bool CameraNode::getScreenshot(std::string screenshotFolderPath, std::string filename, std::string cameraURL)
{
    std::string captureName = screenshotFolderPath + "/" + filename;

    RCLCPP_INFO(LOGGER, "Attempting to capture screenshot from camera: %s", cameraURL.c_str());

    // The URL format will depend on the camera model and configuration
    // std::string camera_url = "rtsp://rover:roverrover@192.168.144.25:554/1/h264major";

    // Open the video stream
    cv::VideoCapture cap(cameraURL);

    if (!cap.isOpened())
    {
        RCLCPP_ERROR(LOGGER, "Failed to open camera stream.");
        return false;
    }

    // Read a single frame
    cv::Mat frame;
    bool ret = cap.read(frame);

    if (ret)
    {
        // Save the frame as a sceenshot:
        cv::imwrite(captureName, frame);
        RCLCPP_INFO(LOGGER, "Screenshot saved successfully as: %s", captureName.c_str());

        // Display the frame
        cv::imshow("IP Camera Screenshot", frame);
        cv::waitKey(0);  // Wait for a key press
        cv::destroyAllWindows();
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Failed to capture frame from camera.");
        return false;
    }

    // Release the video capture object
    cap.release();

    return true;
}

bool CameraNode::startRecording()
{
    return true;
}

bool CameraNode::stopRecording()
{
    return true;
}