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
#include <unordered_map>
#include <thread>
#include <vector>
#include <atomic>
#include <queue>

#include <cstdlib>
#include <sys/stat.h>

#define SCREENSHOT 1
#define VIDEO 2

constexpr uint8_t RECORDING_INTERVAL = 30; //in seconds

/* This folder is used for the functions' declarations */

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

    bool appendRecordings();

    //camera variables
    std::string camURL;
    std::string filename;
    std::string videoFolderPath;

    std::vector<std::string> files;

    uint8_t recordingNumber = 1;
    time_t startTime;

    int frame_width;
    int frame_height;
    double fps;

    //ros logger
    rclcpp::Logger logger_; //allows Recording objects to send logs from ROS nodes

    //cv variables
    cv::VideoCapture cap;
    cv::VideoWriter video_writer;
    cv::Mat frame;


    public:
    Recording(std::string videoFolderPath_in, std::string filename_in, std::string URL_in, rclcpp::Logger logger): camURL(URL_in), filename(filename_in), videoFolderPath(videoFolderPath_in), logger_(logger) {}
    ~Recording()
    {
 
        if (cap.isOpened()) //avoid unnecessary logging when creating temporary objects
        {
            // Release resources
            this->cap.release();
            this->video_writer.release();   

            RCLCPP_INFO(logger_, "Recording stopped.");

            if(appendRecordings())
            {
                RCLCPP_INFO(logger_, "Succesfully appended videos");
            }
            else
            {
                RCLCPP_INFO(logger_, "Something went wrong, couldn't append videos in recording %s", camURL.c_str());
            }
        }
    }
};

class CameraNode : public rclcpp::Node
{
  public:
  private:
    rclcpp::Service<rover_msgs::srv::CameraControl>::SharedPtr _srv_screenshot;
    rclcpp::Service<rover_msgs::srv::CameraControl>::SharedPtr _srv_recording;

    void controlIPCam(const std::shared_ptr<rover_msgs::srv::CameraControl::Request> request,
                      std::shared_ptr<rover_msgs::srv::CameraControl::Response> response);
    std::string getCurrentTime();
    std::string getFileName(const std::string capture_name, std::string camURL, int state);
    std::string getCamID(std::string cameraURL);
    const std::string getFolderPath(int state);
    bool folderExists(const std::string& path);
    bool createFolder(const std::string& path);
    bool getScreenshot(std::string screenshotFolderPath, std::string filename, std::string cameraURL);

    bool isRecording{false};
    bool newRecording(std::string videoFolderPath, std::string filename, std::string cameraURL);
    bool stopRecording(std::string cameraURL);
    void recordingThreadFunction();
    std::thread recordingThread;

    std::unordered_map<std::string, Recording> RecordingMap;

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
        RCLCPP_INFO(LOGGER, "'144.' found.");
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

    RCLCPP_INFO(LOGGER, "Camera ID: %s", camID.c_str());

    return camID;
}

std::string CameraNode::getCurrentTime()
{
    auto now = std::chrono::system_clock::now();  // get system time

    auto now_time = std::chrono::system_clock::to_time_t(now);  // convert to real time

    std::tm tm_now = *std::localtime(&now_time);  // convert to calendar time

    std::stringstream current_time_output;
    current_time_output << std::put_time(&tm_now, "%FT%T");  // ISO 8601 format

    return current_time_output.str();
}

std::string CameraNode::getFileName(const std::string capture_name, std::string camURL, int state)
{
    std::string filename;

    std::string time = getCurrentTime();
    std::string ID = getCamID(camURL);

    switch (state)
    {
        case SCREENSHOT:
            filename = capture_name.empty() ? time + "_" + ID + "_screenshot.png" : time + "_camID:" + ID + "_" + capture_name;
            // Example : 2024-12-10T20:50:00_GPS_25_screenshot.png
            break;

        case VIDEO:
            filename = capture_name.empty() ? time + "_" + ID + "_recording.avi" : time + "_camID:" + ID + "_" + capture_name;
            // Example : 2024-12-10T20:50:00_GPS_25_recording.avi
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

    if (RecordingMap.find(cameraURL) != RecordingMap.end()) //check if currently recording
    {
        Recording* pRecording = &RecordingMap.at(cameraURL); 
        
        // Save the last frame from recording as picture:
            cv::imwrite(captureName, pRecording->getFrame());
            RCLCPP_INFO(LOGGER, "Screenshot saved successfully as: %s", captureName.c_str());

            // Display the frame
            cv::imshow("IP Camera Screenshot", pRecording->getFrame());

        return true;
    }
    else //if not recording proceed normaly
    {
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
}



bool Recording::startRecording()
{
    // Use the provided file name or a default name
    std::string filePath = this->videoFolderPath + "/" + this->filename;
    filePath.insert(filePath.length()-4, '_' + std::to_string(this->recordingNumber++)); //add recording number before .avi
    this->files.push_back(filePath); //add file to list of recordings

    this->cap.open((this->camURL));
    if (!this->cap.isOpened())
    {
        RCLCPP_ERROR(logger_, "Failed to open camera stream.");
        return false;
    }

    // Get frame width and height
    // ChatGPT gave me this, gotta look into it more */
    this->frame_width = static_cast<int>(this->cap.get(cv::CAP_PROP_FRAME_WIDTH));
    this->frame_height = static_cast<int>(this->cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    this->fps = static_cast<double>(this->cap.get(cv::CAP_PROP_FPS));
    
    this->fps = (this->fps > 0) ? fps : 30; //weird bug with usb camera, recording is 2x speed or 1,5x

    RCLCPP_INFO(logger_, "fps set to %f", this->fps);

    // Define the codec and create a VideoWriter object
    /* Also from ChatGPT --> more information on OpenCV
    --> https://docs.opencv.org/4.x/dd/d9e/classcv_1_1VideoWriter.html */

    this->video_writer.open(filePath,
                                 cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                                 this->fps,
                                 cv::Size(frame_width, frame_height));

    if (!video_writer.isOpened())
    {
        RCLCPP_ERROR(logger_, "Error: Could not open the output video file for writing!");
        return false;
    }

    this->startTime = time(0);

    return true;
}

bool Recording::recordFrame()
{
    
    
        this->cap >> this->frame;
        if (this->frame.empty())
            {
                RCLCPP_ERROR(logger_, "Error: Blank frame grabbed!");
                return false;
            }

        // Write frame to the output video file
        this->video_writer.write(this->frame);

        // Show the frame
        //cv::imshow("IP Camera Stream", this->frame);

        if (difftime(time(0), this->startTime) >= RECORDING_INTERVAL) //save every RECORDING_INTERVAL seconds
        {
            std::string filePath = this->videoFolderPath + "/" + this->filename;
            filePath.insert(filePath.length()-4, '_' + std::to_string(this->recordingNumber++));  
            this->files.push_back(filePath); 
            this->video_writer.release();
            
            this->video_writer.open(filePath,
                                 cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                                 this->fps,
                                 cv::Size(this->frame_width, this->frame_height));

            if (!video_writer.isOpened())
            {
                RCLCPP_ERROR(logger_, "Error: Could not open the output video file for writing!");
                return false;
            }    

            this->startTime = time(0);

        }
    
    return true;
}

bool CameraNode::stopRecording(std::string cameraURL)
{
    if (RecordingMap.find(cameraURL) != RecordingMap.end())
    {
        RecordingMap.erase(cameraURL);
        if (RecordingMap.empty())
        {
            this->isRecording = false; //if there are no more recordings: stop the thread
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool CameraNode::newRecording(std::string videoFolderPath, std::string filename, std::string cameraURL)
{
    if (RecordingMap.find(cameraURL) != RecordingMap.end()) //check if recording doesn't already exist
    {
        RCLCPP_ERROR(LOGGER, "Already recording on this stream");
        return false;
    }
    else
    {
        RecordingMap.emplace(cameraURL, Recording(videoFolderPath, filename, cameraURL, LOGGER));

        // Access the recording using at() to safely get the reference
        Recording* pRecording = &RecordingMap.at(cameraURL);
         
        if (pRecording->startRecording()) 
        {
            if (this->isRecording);//if thread is already started: do nothing
            else 
            {
                this->isRecording = true;
                this->recordingThread = std::thread(&CameraNode::recordingThreadFunction, this); //start thread
            }
            return true;
        } 
        else 
        {
            return false;
        }

    }

}

void CameraNode::recordingThreadFunction()
{
    while(this->isRecording)
    {
        for (auto& pair: RecordingMap) //call all active recordings
        {
            if (!pair.second.recordFrame())//if there is an error during the recording stop the faulty recording only
            {
                stopRecording(pair.second.getURL());
            }
        }
    }
}

bool Recording::appendRecordings()
{
    if (this->files.empty())
    {
        return false;
    }
    else
    {
        std::string appendedVideoFilePath = this->videoFolderPath + "/" + this->filename;

        cv::VideoWriter appender(appendedVideoFilePath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                                 this->fps,
                                 cv::Size(this->frame_width, this->frame_height));

        if (!appender.isOpened()) 
        {
            RCLCPP_ERROR(logger_, "Couldn't launch video appender writer");
            return false;
        }


        for (const auto& current_file: files)
        {
            
            cv::VideoCapture cap(current_file);

            if (!cap.isOpened())
            {
                RCLCPP_ERROR(logger_, "file: %s was empty", current_file.c_str());
                continue; // empty file; skip
            }    

            RCLCPP_INFO(logger_, "Appending file %s", current_file.c_str());


            cv::Mat frame;
            while (cap.read(frame)) // Read each frame
            {  
                appender.write(frame); 
            } 
        
            cap.release();

            RCLCPP_INFO(logger_, "Appending of %s complete", current_file.c_str());
            
        }

        appender.release();
        return true;    
    }
    
}