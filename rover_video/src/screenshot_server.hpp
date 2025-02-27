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
    std::string getCurrentTime();
    std::string getFileName(const std::string capture_name, int state);
    const std::string getFolderPath(int state);
    bool folderExists(const std::string& path);
    bool createFolder(const std::string& path);
    bool getScreenshot(std::string screenshotFolderPath, std::string filename, std::string cameraURL);
    bool startRecording(std::string videoFolderPath, std::string filename, std::string cameraURL);
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

std::string CameraNode::getFileName(const std::string capture_name, int state)
{
    std::string filename;

    switch (state)
    {
        case SCREENSHOT: filename = capture_name.empty() ? getCurrentTime() + "_screenshot.png" : capture_name; break;

        case VIDEO: filename = capture_name.empty() ? getCurrentTime() + "_recording.avi" : capture_name; break;
    }
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

bool CameraNode::startRecording(std::string videoFolderPath, std::string filename, std::string cameraURL)
{
    // Use the provided file name or a default name
    std::string filePath = videoFolderPath + "/" + filename;

    cv::VideoCapture cap(cameraURL);

    if (!cap.isOpened())
    {
        RCLCPP_ERROR(LOGGER, "Failed to open camera stream.");
        return false;
    }

    // Get frame width and height
    // ChatGPT gave me this, gotta look into it more */
    int frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    int fps = static_cast<int>(cap.get(cv::CAP_PROP_FPS));

    // Define the codec and create a VideoWriter object
    /* Also from ChatGPT --> more information on OpenCV
    --> https://docs.opencv.org/4.x/dd/d9e/classcv_1_1VideoWriter.html */

    cv::VideoWriter video_writer(filePath,
                                 cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                                 (fps > 0 ? fps : 30),
                                 cv::Size(frame_width, frame_height));

    if (!video_writer.isOpened())
    {
        RCLCPP_ERROR(LOGGER, "Error: Could not open the output video file for writing!");
        return false;
    }

    RCLCPP_INFO(LOGGER, "Recording... Press 'q' to stop.");

    cv::Mat frame;
    for (EVER)
    {
        cap >> frame;
        if (frame.empty())
        {
            RCLCPP_ERROR(LOGGER, "Error: Blank frame grabbed!");
            return false;
        }

        // Write frame to the output video file
        video_writer.write(frame);

        // Show the frame
        cv::imshow("IP Camera Stream", frame);

        // Press 'q' to exit
        if (cv::waitKey(1) == 'q')
        {
            break;
        }
    }

    // Release resources
    cap.release();
    video_writer.release();
    cv::destroyAllWindows();

    RCLCPP_INFO(LOGGER, "Recording stopped.");
    return true;
}

bool CameraNode::stopRecording()
{
    return true;
}