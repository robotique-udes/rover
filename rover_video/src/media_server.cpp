#include "rclcpp/rclcpp.hpp"
#include "rovus_lib/macros.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

#include <cstdlib>
#include <sys/stat.h>

std::string selectCam(int camID)
{
    std::string camURL = "";

    while(camURL == "")
    {
        switch(camID)
        {
            case 25:
                camURL = "rtsp://rover:roverrover@192.168.144.25:554/1/h264major";
                break;
            
            case 40:
                camURL = "rtsp://rover:roverrover@192.168.144.40:554/1/h264major";
                break;

            default:
                std::cout << "Invalid camera ID" << std::endl;
                std::cout << "Enter new camera ID: ";
                std::cin >> camID;
                break;

        }
    }
    
    return camURL;
}

int screenshotIPCam(std::string cameraURL, std::string folder) 
{
    // The URL format will depend on the camera model and configuration
    //std::string camera_url = "rtsp://rover:roverrover@192.168.144.25:554/1/h264major";

    // Open the video stream
    cv::VideoCapture cap(cameraURL);

    if (!cap.isOpened()) 
    {
        std::cerr << "Error: Unable to access the camera stream." << std::endl;
        return -1;
    }

    // Read a single frame
    cv::Mat frame;
    bool ret = cap.read(frame);

    if (ret) 
    {
        
        // Save the frame as a sceenshot:
        std::string filename = folder + "/ip_camera_screenshot.png";
        cv::imwrite(filename, frame);
        std::cout << "Screenshot saved as " << filename << std::endl;
        

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

int recordingIpCam(std::string cameraURL, std::string folder)
{
    cv::VideoCapture cap(cameraURL);

    if(!cap.isOpened())
    {
        std::cerr << "Error: Unable to access the camera stream." << std::endl;
        return -1;
    }


    // Get frame width and height   
    /* ChatGPT gave me this, gotta look into it more */
    int frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    int fps = static_cast<int>(cap.get(cv::CAP_PROP_FPS));

    std::string path_to_folder = folder + "/ip_cam_recording.avi";

    // Define the codec and create a VideoWriter object     
    /* Also from ChatGPT --> more information on OpenCV 
    --> https://docs.opencv.org/4.x/dd/d9e/classcv_1_1VideoWriter.html */
    cv::VideoWriter video_writer(path_to_folder, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
    (fps > 0 ? fps : 30), cv::Size(frame_width, frame_height));

    if(!video_writer.isOpened())
    {
        std::cerr << "Error: Could not open the output video file for writing!" << std::endl;
    }

    std::cout << "Recording... Press 'q' to stop." << std::endl;

    cv::Mat frame;
    for(EVER) 
    {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error: Blank frame grabbed!" << std::endl;
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

    std::cout << "Recording stopped." << std::endl;

    return 0;
}

// Verifies if screenshot folder already exists
bool folderExists(const std::string& path)    
{
    struct stat info;
    return (stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR)); // I dont exactly understand this part
}

// Creating screenshot Folder if doesnt already exists
bool createFolder(const std::string& path)    
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



int main()
{
    int camID = 0;
    int status = 0;

    std::string dir = GET_PACKAGE_SOURCE_DIR("rover_video"); // finds the path to our package 
    const std::string screenshotFolderPath = std::string(dir) + "/src/screenshots"; // Path necessary for the screenshots folder /* Look for const in doc, i think they are illegal in Rovus */
    const std::string recordingFolderPath = std::string(dir) + "/src/recordings";   // Path necessary for the recordings folder

    createFolder(screenshotFolderPath);
    createFolder(recordingFolderPath);

    std::cout << "Enter camera ID:" << std::endl;
    std::cin >> camID;
    std::string cameraURL = selectCam(camID);

   while(status != 'e') // Keeps printing the menu until the user wants to quit
   {
        std::cout << "What do you want to do?" << std::endl;
        std::cout << "1: Screenshot" << std::endl << "2: Recording" << std::endl << "3: Exit" << std::endl;
        std::cin >> status;

        switch (status) // Basic switch case to create a menu using the different functions
        {
            case 1:
                screenshotIPCam(cameraURL, screenshotFolderPath);
                status = 0;
                break;

            case 2:
                recordingIpCam(cameraURL, recordingFolderPath);
                status = 0;
                break;

            case 3: 
                status = 'e';
                break;
                
            default:
                std::cout << "Invalid option." << std::endl;
                status = 0;
                break;
        }
    
    }
    return 0;
}