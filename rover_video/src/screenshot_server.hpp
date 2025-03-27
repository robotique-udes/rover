#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/gps_position.hpp"
#include "rover_msgs/srv/camera_control.hpp"
#include "rovus_lib/macros.h"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <cstdlib>
#include <sys/stat.h>

#define SCREENSHOT 1
#define VIDEO 2

constexpr uint8_t RECORDING_INTERVAL = 20;  // in seconds

/* This folder is used for the functions' declarations */

/*
std::string getTempdir()
{
    const char* temp = std::getenv("TMPDIR");
    if (!temp)
        temp = std::getenv("TMP");
    if (!temp)
        temp = std::getenv("TEMP");
    if (!temp)
        temp = "/tmp";  // Default for Ubuntu
    return std::string(temp);
}
*/

class Recording
{
  public:
    bool startRecording();
    bool recordFrame();

    std::string getURL()
    {
        return this->camURL;
    }

    cv::Mat getFrame()
    {
        return this->frame;
    }

    uint8_t getFPS()
    {
        return this->fps;
    }

  private:
    void RecordingThreadFunction();
    std::function<void(std::string)> RequestShutdown_;

    std::shared_ptr<std::thread> recordingThread;
    std::atomic<bool> stopRecording{false};

    // camera variables
    std::string camURL;
    std::string pipeline;
    std::string filename;
    std::string videoFolderPath;

    std::vector<std::string> files;

    uint8_t recordingNumber = 1;
    time_t startTime;

    int frame_width;
    int frame_height;
    double fps;

    // ros logger
    std::shared_ptr<rclcpp::Logger> logger_;  // allows Recording objects to send logs from ROS nodes

    // cv variables
    cv::VideoCapture cap;
    cv::VideoWriter video_writer;
    cv::VideoWriter appender;
    cv::Mat frame;

  public:
    Recording(std::string videoFolderPath_in,
              std::string filename_in,
              std::string URL_in,
              std::shared_ptr<rclcpp::Logger> logger,
              std::function<void(std::string)> RequestShutdown):
        RequestShutdown_(RequestShutdown),
        camURL(URL_in),
        filename(filename_in),
        videoFolderPath(videoFolderPath_in),
        logger_(logger)
    {
    }

    Recording(Recording&& other) noexcept:
        // move constructor, used to move the temporary object created by hashmap emplace
        RequestShutdown_(std::move(other.RequestShutdown_)),
        recordingThread(std::move(other.recordingThread)),
        camURL(std::move(other.camURL)),
        pipeline(std::move(other.pipeline)),
        filename(std::move(other.filename)),
        videoFolderPath(std::move(other.videoFolderPath)),
        files(std::move(other.files)),
        recordingNumber(other.recordingNumber),
        startTime(other.startTime),
        frame_width(other.frame_width),
        frame_height(other.frame_height),
        fps(other.fps),
        logger_(std::move(other.logger_)),
        cap(std::move(other.cap)),
        video_writer(std::move(other.video_writer)),
        appender(std::move(other.appender)),
        frame(std::move(other.frame))
    {
        stopRecording.store(other.stopRecording.load());  // cannot move atomic
    }

    Recording& operator=(Recording&& other) noexcept
    {  // move operator just to be safe
        if (this != &other)
        {  // Prevent self-assignment

            // Move resources
            RequestShutdown_ = std::move(other.RequestShutdown_);
            recordingThread = std::move(other.recordingThread);
            stopRecording.store(other.stopRecording.load(std::memory_order_acquire), std::memory_order_release);

            camURL = std::move(other.camURL);
            pipeline = std::move(other.pipeline);
            filename = std::move(other.filename);
            videoFolderPath = std::move(other.videoFolderPath);
            files = std::move(other.files);
            recordingNumber = other.recordingNumber;
            startTime = other.startTime;
            frame_width = other.frame_width;
            frame_height = other.frame_height;
            fps = other.fps;
            logger_ = std::move(other.logger_);

            appender = std::move(other.appender);
            cap = std::move(other.cap);  // Move cv ressources
            video_writer = std::move(other.video_writer);
            frame = std::move(other.frame);
        }
        return *this;
    }

    ~Recording()
    {
        if (cap.isOpened())  // avoid unnecessary logging when creating temporary objects
        {
            stopRecording.store(true);

            if (recordingThread->joinable())
            {
                recordingThread->join();
            }
            // Release resources
            this->cap.release();
            this->video_writer.release();
            this->appender.release();

            RCLCPP_INFO(*logger_, "Recording stopped.");
        }
    }
};

class CameraNode : public rclcpp::Node
{
  public:
  private:
    rclcpp::Service<rover_msgs::srv::CameraControl>::SharedPtr _srv_control;
    rclcpp::Subscription<rover_msgs::msg::GpsPosition>::SharedPtr _msg_position;
    float last_latitude = 0.0, last_longitude = 0.0;

    void controlIPCam(const rover_msgs::srv::CameraControl::Request& request, rover_msgs::srv::CameraControl::Response& response);
    std::string getCurrentTime();
    std::string getFileName(const std::string capture_name, std::string camURL, int state);
    std::string getCamID(std::string cameraURL);
    void callbackPosition(const rover_msgs::msg::GpsPosition& gps_message);
    const std::string getFolderPath(int state);
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

std::string CameraNode::getCamID(std::string cameraURL)
{
    std::string camID;

    std::string::size_type nextDotPos;
    std::string::size_type posID = cameraURL.find("144.");

    if (posID != std::string::npos)
    {
        RCLCPP_DEBUG(LOGGER, "'144.' found.");
        posID += 4;
        nextDotPos = cameraURL.find(':', posID);

        if (nextDotPos != std::string::npos)
        {
            camID = cameraURL.substr(posID, nextDotPos - posID);
        }
        else
        {
            camID = cameraURL.substr(posID);
        }
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "'144.' not found.");
    }

    RCLCPP_DEBUG(LOGGER, "Camera ID: %s", camID.c_str());

    return camID;
}

// get current time in ISO format for filename
std::string CameraNode::getCurrentTime()
{
    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();  // get system time

    std::time_t now_time = std::chrono::system_clock::to_time_t(now);  // convert to real time

    std::tm tm_now = *std::localtime(&now_time);  // convert to calendar time

    std::stringstream current_time_output;
    current_time_output << std::put_time(&tm_now, "%FT%T");  // ISO 8601 format

    return current_time_output.str();
}

std::string CameraNode::getFileName(const std::string capture_name, std::string camURL, int state)
{
    std::string filename;

    std::string time = getCurrentTime();
    std::string latitude = std::to_string(last_latitude);
    std::string longitude = std::to_string(last_longitude);
    std::string ID = getCamID(camURL);

    switch (state)
    {
        case SCREENSHOT:
            filename = capture_name.empty()
                           ? time + "_lat:" + latitude + "_long:" + longitude + "_" + ID + "_screenshot.png"
                           : time + "_lat:" + latitude + "_long:" + longitude + "_camID:" + ID + "_" + capture_name;
            // Example : 2024-12-10T20:50:00_GPS_30_screenshot.png
            break;

        case VIDEO:
            filename = capture_name.empty()
                           ? time + "_lat:" + latitude + "_long:" + longitude + "_" + ID + "_recording.avi"
                           : time + "_lat:" + latitude + "_long:" + longitude + "_camID:" + ID + "_" + capture_name;
            // Example : 2024-12-10T20:50:00_GPS_30_recording.avi
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

    RCLCPP_DEBUG(LOGGER, "Directory already exists: %s", path.c_str());
    return true;
}

// capture screenshot either from recording stream (if it exist) or from a new stream
bool CameraNode::getScreenshot(std::string screenshotFolderPath, std::string filename, std::string cameraURL)
{
    std::string captureName = screenshotFolderPath + "/" + filename;

    RCLCPP_INFO(LOGGER, "Attempting to capture screenshot from camera: %s", cameraURL.c_str());

    // The URL format will depend on the camera model and configuration
    // std::string camera_url = "rtsp://rover:roverrover@192.168.144.30:554/1/h264major";

    // Open the video stream

    if (RecordingMap.find(cameraURL) != RecordingMap.end())  // check if currently recording
    {
        Recording* pRecording = &RecordingMap.at(cameraURL);

        // Save the last frame from recording as picture:
        cv::imwrite(captureName, pRecording->getFrame());
        RCLCPP_INFO(LOGGER, "Screenshot saved successfully as: %s", captureName.c_str());

        // Display the frame
        cv::imshow("IP Camera Screenshot", pRecording->getFrame());

        return true;
    }
    else  // if not recording proceed normaly
    {
        std::string pipeline
            = "rtspsrc location=" + cameraURL
              + " latency=0 drop=true ! decodebin ! videorate max-rate=30 ! videoconvert ! queue max-size-buffers=1 ! appsink";

        cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

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

            // Display the frame for debug
            // cv::imshow("IP Camera Screenshot", frame);
            // cv::waitKey(0);  // Wait for a key press
            // cv::destroyAllWindows();
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
}

// Start the video recording and initialize all openCV members
bool Recording::startRecording()
{
    // Getting the directory for the recording
    std::string filePath = this->videoFolderPath + "/" + this->filename;
    filePath.insert(filePath.length() - 4, '_' + std::to_string(this->recordingNumber++));  // add recording number before .avi
    this->files.push_back(filePath);                                                        // add file to list of recordings

    this->pipeline
        = "rtspsrc location=" + this->camURL
          + " latency=0 drop=true ! decodebin ! videorate max-rate=30 ! videoconvert ! queue max-size-buffers=1 ! appsink";

    this->cap.open(this->pipeline, cv::CAP_GSTREAMER);
    if (!this->cap.isOpened())
    {
        RCLCPP_ERROR(*logger_, "Failed to open camera stream.");
        return false;
    }

    // Get frame width and height

    this->frame_width = static_cast<int>(this->cap.get(cv::CAP_PROP_FRAME_WIDTH));
    this->frame_height = static_cast<int>(this->cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    this->fps = static_cast<double>(this->cap.get(cv::CAP_PROP_FPS));

    this->fps = (this->fps > 0) ? fps : 30;  // weird bug with usb camera, recording is 2x speed or 1,5x

    RCLCPP_DEBUG(*logger_, "fps set to %f", this->fps);

    // Define the codec and create a VideoWriter object
    /* More information on OpenCV --> https://docs.opencv.org/4.x/dd/d9e/classcv_1_1VideoWriter.html */

    this->video_writer.open(filePath,
                            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                            this->fps,
                            cv::Size(frame_width, frame_height));

    if (!video_writer.isOpened())
    {
        RCLCPP_ERROR(*logger_, "Error: Could not open the output video file for writing!");
        return false;
    }

    std::string appendedVideoFilePath = this->videoFolderPath + "/" + this->filename;

    appender = cv::VideoWriter(appendedVideoFilePath,
                               cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                               this->fps,
                               cv::Size(this->frame_width, this->frame_height));

    if (!appender.isOpened())
    {
        RCLCPP_ERROR(*logger_, "Couldn't launch video sticher");
        return false;
    }

    this->startTime = time(0);

    recordingThread = std::make_shared<std::thread>([this]() { RecordingThreadFunction(); });

    RCLCPP_INFO(*logger_, "Recording started for stream %s", camURL.c_str());

    return true;
}

// Capture 1 frame and write it to the short save and long save
// Called from the recordingThread
bool Recording::recordFrame()
{
    if (!this->cap.isOpened())
    {
        if (!this->stopRecording.load())
            RCLCPP_ERROR(*logger_, "Error: cap is closed");
        return false;
    }

    this->cap >> this->frame;
    if (this->frame.empty())
    {
        if (!this->stopRecording.load())
            RCLCPP_ERROR(*logger_, "Error: Blank frame grabbed!");
        return false;
    }

    if (!this->video_writer.isOpened() || !this->appender.isOpened())
    {
        if (!this->stopRecording.load())
            RCLCPP_ERROR(*logger_, "Error: video writer is closed");
        return false;
    }
    // Write frame to the output video file
    this->video_writer.write(this->frame);
    this->appender.write(this->frame);

    // Show the frame
    // cv::imshow("IP Camera Stream", this->frame);

    if (difftime(time(0), this->startTime) >= RECORDING_INTERVAL)  // save every RECORDING_INTERVAL seconds
    {
        std::string filePath = this->videoFolderPath + "/" + this->filename;
        filePath.insert(filePath.length() - 4, '_' + std::to_string(this->recordingNumber++));
        this->files.push_back(filePath);
        this->video_writer.release();

        this->video_writer.open(filePath,
                                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                                this->fps,
                                cv::Size(this->frame_width, this->frame_height));

        if (!video_writer.isOpened())
        {
            RCLCPP_ERROR(*logger_, "Error: Could not open the output video file for writing!");
            return false;
        }

        this->startTime = time(0);
    }

    return true;
}

// call the recording destroyer and erase it from the hashmap
bool CameraNode::stopRecording(std::string cameraURL)
{
    std::lock_guard<std::mutex> lock(recordingMutex);

    if (RecordingMap.find(cameraURL) != RecordingMap.end())
    {
        RecordingMap.erase(cameraURL);

        if (RecordingMap.empty())
        {
            watchDogStop.store(true);
            recordingCv.notify_one();
        }
        return true;
    }
    else
    {
        return false;
    }
}

// add a new recording to the hashmap, check it's working and start the watch dog if it,s not already started
bool CameraNode::newRecording(std::string videoFolderPath, std::string filename, std::string cameraURL)
{
    if (RecordingMap.find(cameraURL) != RecordingMap.end())  // check if recording doesn't already exist
    {
        return false;
    }
    else
    {
        RecordingMap.emplace(cameraURL,
                             Recording(videoFolderPath,
                                       filename,
                                       cameraURL,
                                       std::make_shared<rclcpp::Logger>(LOGGER),
                                       [this](std::string url) { RequestShutdown(url); }));

        if (!videoWatchDog.joinable())
        {
            StartWatchDog();
        }

        // Access the recording using at() to safely get the reference
        Recording* pRecording = &RecordingMap.at(cameraURL);

        if (pRecording->startRecording())
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

// gps position for file name
void CameraNode::callbackPosition(const rover_msgs::msg::GpsPosition& gps_message)
{
    last_latitude = gps_message.latitude;
    last_longitude = gps_message.longitude;
}

// call recordFrame and use callback function (shutdown request) in case of error
void Recording::RecordingThreadFunction()
{
    while (!this->stopRecording.load())
    {
        if (!recordFrame() && !this->stopRecording.load())  // if error execept on last loop
        {
            RCLCPP_WARN(*logger_, "Requesting shutdown for %s", this->camURL.c_str());
            RequestShutdown_(this->camURL);
            this->stopRecording.store(true);
        }
    }
    return;
}

// Add recording key to shutdown list and notify watch dog for shutdown
void CameraNode::RequestShutdown(std::string camURL)
{
    RCLCPP_WARN(LOGGER, "Received shutdown request for %s", camURL.c_str());

    {
        std::unique_lock<std::mutex> lock(recordingMutex);
        RecordingShutdownRequestSet.insert(camURL);
    }  // unlock
    recordingCv.notify_one();
    return;
}

// start the VideoWatchDogFunction
bool CameraNode::StartWatchDog()
{
    watchDogStop.store(false);
    videoWatchDog = std::thread(&CameraNode::VideoWatchDogFunction, this);
    return true;
}

// when requested, shutdown and erase recordings that were in error from hashmap
void CameraNode::VideoWatchDogFunction()
{
    RCLCPP_DEBUG(LOGGER, "Starting video watchdog");
    while (!watchDogStop.load())
    {
        std::unique_lock<std::mutex> lock(recordingMutex);
        recordingCv.wait(lock, [this] { return watchDogStop.load() || !RecordingShutdownRequestSet.empty(); });

        if (watchDogStop)
            break;
        else
        {
            for (std::string url : RecordingShutdownRequestSet)
            {
                RCLCPP_WARN(LOGGER, "Processing Shutdown for %s", url.c_str());
                if (RecordingMap.find(url) != RecordingMap.end())
                {
                    if (!RecordingMap.erase(url))
                    {
                        RCLCPP_ERROR(LOGGER, "Shutdown request for %s could not be processed, please try again", url.c_str());
                    }

                    if (RecordingMap.empty())
                    {
                        watchDogStop.store(true);
                    }
                }
                else
                {
                    RCLCPP_ERROR(LOGGER, "Unable to find %s for shutdown, please try again", url.c_str());
                }
            }

            RecordingShutdownRequestSet.clear();
        }
    }
    RCLCPP_DEBUG(LOGGER, "Stopping video watchdog");
    return;
}