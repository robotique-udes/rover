#include "rclcpp/rclcpp.hpp"
#include "rovus_lib/macros.h"
#include "rover_msgs/srv/CameraControl.hpp"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>


#include <cstdlib>
#include <sys/stat.h>

class CameraNode : public rclcpp::Node
{
    public:

    private:
    std::string cameraURL = "";

    //Gotta ask Philippe if i can do this
    std::string dir = GET_PACKAGE_SOURCE_DIR("rover_video"); // finds the path to our package 
    // Path necessary for the screenshots folder 
    /* Look for const in doc, i think they are illegal in Rovus */
    const std::string screenshotFolderPath = std::string(dir) + "/src/screenshots";
    // Path necessary for the recordings folder
    const std::string recordingFolderPath = std::string(dir) + "/src/recordings";   

    std::string selectCameraURL(int camID);
    void screenshotIPCam(const std::shared_ptr<rover_msgs::srv::CameraControl::Request> request,
    std::shared_ptr<rover_msgs::srv::ScreenshotControl::Response> response); 
    void recordingIPCam(const std::shared_ptr<rover_msgs::srv::CameraControl::Request> request,
    std::shared_ptr<rover_msgs::srv::ScreenshotControl::Response> response);
    bool folderExists(const std::string& path);
    bool createFolder(const std::string& path);

    public:
    CameraNode();
    ~CameraNode() {}
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        rclcpp::spin(std::make_shared<CameraNode>());
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("Dead Node"), "Killing node on exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}

CameraNode::CameraNode(): Node("media_server")
{
    // Default value set to 0, probably will change it later
    this->declare_parameter<int>("cameraID", 0);
    int cameraID = this->get_parameter("cameraID").as_int();
    RCLCPP_INFO_ONCE(this->get_logger(), "Camera ID: %d", cameraID);

    _srv_screenshot = this->create_service<rover_msgs::srv::CameraControl>(
        "screenshot",
        std::bind(&CameraNode::screenshotIPCam, this, std::placeholders::_1, std::placeholders::_2));

    _srv_recording = this->create_service<rover_msgs::srv::CameraControl>(
        "recording",
        std::bind(&CameraNode::recordingIPCam, this, std::placeholders::_1, std::placeholders::_2));

}

std::string CameraNode::selectCameraURL(int camID)
{
    // Rethink the while(camURL == "") loop
    std::string camURL = "";

        switch(camID)
        {
            case 25:
                camURL = "rtsp://rover:roverrover@192.168.144.25:554/1/h264major";
                break;
            
            case 40:
                camURL = "rtsp://rover:roverrover@192.168.144.40:554/1/h264major";
                break;

            default:
                RCLCPP_ERROR(LOGGER, "Invalid Camera ID");
                break;

        }

    return camURL;

}

std::string get_current_time()
{
    auto now = std::chrono::system_clock::now(); //get system time

    auto now_time = std::chrono::system_clock::to_time_t(now); //convert to real time
    
    std::tm tm_now = *std::localtime(&now_time); //convert to calendar time
    
    std::stringstream current_time_output;
    current_time_output << std::put_time(&tm_now, "%FT%T");  // ISO 8601 format  

    return current_time_output.str();  
} 


void CameraNode::screenshotIPCam( const std::shared_ptr<rover_msgs::srv::ScreenshotControl::Request> request,
    std::shared_ptr<rover_msgs::srv::ScreenshotControl::Response> response) 
{
    
    // The URL format will depend on the camera model and configuration
    //std::string camera_url = "rtsp://rover:roverrover@192.168.144.25:554/1/h264major";

    // Use the provided file name or a default name
    std::string filename = request->file_name.empty() ? get_current_time() + "_screenshot.png" : request->file_name;
    std::string filePath = screenshotFolderPath + "/" + filename;

    if(!createFolder(screenshotFolderPath))
    {
        RCLCPP_ERROR(LOGGER, "Failed to create screenshots folder or it already exists.");
        response->success = false;
    }

    // Open the video stream
    cv::VideoCapture cap(cameraURL);

    if (!cap.isOpened()) 
    {
        RCLCPP_ERROR(LOGGER "Failed to open camera stream.");
        response->success = false; //what does it do??
    }


    // Read a single frame
    cv::Mat frame;
    bool ret = cap.read(frame);

    if (ret) 
    {
        
        // Save the frame as a sceenshot:
        cv::imwrite(filename, frame);
        RCLCPP_INFO(LOGGER, "Screenshot saved as %s", filename);
        

        // Display the frame
        cv::imshow("IP Camera Screenshot", frame);
        cv::waitKey(0); // Wait for a key press
        cv::destroyAllWindows();
    } 
    else 
    {
        RCLCPP_ERROR(LOGGER, "Failed to capture frame.");
        response->success = false;
    }

    // Release the video capture object
    cap.release();

}

void CameraNode::recordingIPCam(const std::shared_ptr<rover_msgs::srv::ScreenshotControl::Request> request,
    std::shared_ptr<rover_msgs::srv::ScreenshotControl::Response> response)
{
    // Select the correct URL using the internal function
    std::string cameraURL = selectCameraURL(request->cameraID);

    // Use the provided file name or a default name
    std::string filename = request->file_name.empty() ? get_current_time() + "_recording.avi" : request->file_name;
    std::string filePath = recordingFolderPath + "/" + filename;

    cv::VideoCapture cap(cameraURL);

        if(!createFolder(recordingFolderPath))
    {
        RCLCPP_ERROR(LOGGER, "Failed to create screenshots folder or it already exists.");
        response->success = false;
    }

    if(!cap.isOpened())
    {
        RCLCPP_ERROR(LOGGER, "Failed to open camera stream.");
        response->success = false;
    }


    // Get frame width and height   
    /* ChatGPT gave me this, gotta look into it more */
    int frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    int fps = static_cast<int>(cap.get(cv::CAP_PROP_FPS));

    std::string pathToFolder = folder + "/ip_cam_recording.avi";

    // Define the codec and create a VideoWriter object     
    /* Also from ChatGPT --> more information on OpenCV 
    --> https://docs.opencv.org/4.x/dd/d9e/classcv_1_1VideoWriter.html */
    cv::VideoWriter video_writer(pathToFolder, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
    (fps > 0 ? fps : 30), cv::Size(frame_width, frame_height));

    if(!video_writer.isOpened())
    {
        RCLCPP_ERROR(LOGGER, "Error: Could not open the output video file for writing!");
        response->success = false;
    }

    RCLCPP_INFO(LOGGER, "Recording... Press 'q' to stop.");

    cv::Mat frame;
    for(EVER) 
    {
        cap >> frame;
        if (frame.empty()) 
        {
            RCLCPP_ERROR(LOGGER, "Error: Blank frame grabbed!");
            break;
        }

        // Write frame to the output video file
        video_writer.write(frame);

        // Show the frame
        cv::imshow("IP Camera Stream", frame);

        // Press 'q' to exit
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // Release resources
    cap.release();
    video_writer.release();
    cv::destroyAllWindows();

    RCLCPP_INFO(LOGGER, "Recording stopped.");

}

// Verifies if screenshot folder already exists
bool CameraNode::folderExists(const std::string& path)    
{
    struct stat info;
    return (stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR)); // I dont exactly understand this part
}

// Creating screenshot Folder if doesnt already exists
bool CameraNode::createFolder(const std::string& path)    
{
    if (!folderExists(path)) 
    {
        if (mkdir(path.c_str(), 0777) == 0) 
        {  // 0777 = Full permissions
            // std::cout << "Directory created: " << path << std::endl;
            return true;
        } 
        else 
        {
            RCLCPP_INFO(LOGGER, "mkdir failed");
            return false;
        }
    }
   // std::cout << "Directory already exists: " << path << std::endl;
    return true;
}