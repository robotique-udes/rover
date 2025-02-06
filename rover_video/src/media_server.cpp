#include "rclcpp/rclcpp.hpp"
#include "rovus_lib/macros.h"
#include "rover_msgs/msg/Screenshot.hpp"
#include "rover_msgs/msg/Recording.hpp"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

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
    int screenshotIPCam(std::string cameraURL, std::string folder); 
    int recordingIPCam(std::string cameraURL, std::string folder);
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

    _srv_screenshot = this->create_service<rover_msgs::srv::Screenshot>(
        "screenshot",
        std::bind(&CameraNode::screenshotIPCam, this, std::placeholders::_1, std::placeholders::_2))

    _srv_recording = this->create_service<rover_msgs::srv::Recording>(
        "recording",
        std::bind(&CameraNode::recordingIPCam, this, std::placeholders::_1, std::placeholders::_2))

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

int CameraNode::screenshotIPCam(std::string cameraURL, std::string folder) 
{
    // The URL format will depend on the camera model and configuration
    //std::string camera_url = "rtsp://rover:roverrover@192.168.144.25:554/1/h264major";

    // Open the video stream
    cv::VideoCapture cap(cameraURL);

    if(!createFolder(folder))
    {
        RCLCPP_ERROR(LOGGER, "Failed to create screenshots folder or it already exists.");
        response->success = false;
        return;
    }

    if (!cap.isOpened()) 
    {
        RCLCPP_ERROR(LOGGER "Failed to open camera stream.");
        response->success = false; //what does it do??
        return; //necessary??
    }

    // Read a single frame
    cv::Mat frame;
    bool ret = cap.read(frame);

    if (ret) 
    {
        
        // Save the frame as a sceenshot:
        std::string filename = folder + "/ip_camera_screenshot.png";
        cv::imwrite(filename, frame);
        RCLCPP_INFO(LOGGER, "Screenshot saved as %s", filename);
        

        // Display the frame
        cv::imshow("IP Camera Screenshot", frame);
        cv::waitKey(0); // Wait for a key press
        cv::destroyAllWindows();
    } 
    else 
    {
        std::cerr << "Error: Unable to capture a frame." << std::endl;
    }

    // Release the video capture object
    cap.release();

    return 0;
}

int CameraNode::recordingIPCam(std::string cameraURL, std::string folder)
{
    cv::VideoCapture cap(cameraURL);

        if(!createFolder(folder))
    {
        RCLCPP_ERROR(LOGGER, "Failed to create screenshots folder or it already exists.");
        response->success = false;
        return;
    }

    if(!cap.isOpened())
    {
        RCLCPP_ERROR(LOGGER, "Failed to open camera stream.");
        response->success = false;
        return;
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

    return 0;
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
            perror("mkdir failed");  // Prints error if mkdir fails
            return false;
        }
    }
   // std::cout << "Directory already exists: " << path << std::endl;
    return true;
}