#include "rclcpp/rclcpp.hpp"
#include "rovus_lib/macros.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

#include <cstdlib>
#include <sys/stat.h>

std::string selectCam(int camID)
{
    std::string cam_url = "";

    while(cam_url == "")
    {
        switch(camID)
        {
            case 25:
                cam_url = "rtsp://rover:roverrover@192.168.144.25:554/1/h264major";
                break;
            
            case 40:
                cam_url = "rtsp://rover:roverrover@192.168.144.40:554/1/h264major";

                break;

            default:
                std::cout << "Invalid camera ID" << std::endl;
                std::cout << "Enter new camera ID: ";
                std::cin >> camID;
                break;

        }
    }
    
    return cam_url;
}

int screenshotIPCam(std::string camera_url) {
    // The URL format will depend on the camera model and configuration
    //std::string camera_url = "rtsp://rover:roverrover@192.168.144.25:554/1/h264major";

    // Open the video stream
    cv::VideoCapture cap(camera_url);

    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to access the camera stream." << std::endl;
        return -1;
    }

    // Read a single frame
    cv::Mat frame;
    bool ret = cap.read(frame);

    if (ret) {
        

        // Finds the environmental variable HOME (Assuming everyone has its ros2_ws folder in its home, for now)
        std::string dir = getenv("HOME"); //Comble les variables d'environnement
        std::cout << "env:HOME ==> " << dir << std::endl;
        std::string dir2 = GET_PACKAGE_SOURCE_DIR("rover_video");      /* Need to ask Philippe how does it work */
        // std::cout << "rover_video directory: ==> " << dir2 << std::endl;
        


        // Save the frame as a screenshot

        /* Test with GET_PACKAGE_SOURCE_DIR*/
        // std::string filename2 = std::string(dir2) + "/screenshots/ip_camera_screenshot";
        // std::cout << "Screenshot 2 saved as " << filename2 << std::endl;

        std::string filename = std::string(dir) + "/ros2_ws/src/rover/rover_video/src/screenshots/ip_camera_screenshot.jpg";
        cv::imwrite(filename, frame);
        std::cout << "Screenshot saved as " << filename << std::endl;
        

        // Display the frame
        cv::imshow("IP Camera Screenshot", frame);
        cv::waitKey(0); // Wait for a key press
        cv::destroyAllWindows();
    } else {
        std::cerr << "Error: Unable to capture a frame." << std::endl;
    }

    // Release the video capture object
    cap.release();

    return 0;
}

// Verifies if screenshot folder already exists
bool screenshotFolderExists(const std::string& path)    
{
    struct stat info;
    return (stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR));
}

// Creating screenshot Folder if doesnt already exists
bool screenshotFolder(const std::string& path)    
{
        if (!screenshotFolderExists(path)) {
        if (mkdir(path.c_str(), 0777) == 0) {  // 0777 = Full permissions
            std::cout << "Directory created: " << path << std::endl;
            return true;
        } else {
            perror("mkdir failed");  // Prints error if mkdir fails
            return false;
        }
    }
    std::cout << "Directory already exists: " << path << std::endl;
    return true;
}



int main()
{
    int cam_id = 0;

    std::string HOME = getenv("HOME"); //Comble les variables d'environnement
    std::string screenshot_folder_path = std::string(HOME) + "/ros2_ws/src/rover/rover_video/src/screenshots";

    screenshotFolder(screenshot_folder_path);

    std::cout << "Enter camera ID:" << std::endl;
    std::cin >> cam_id;
    std::string camera_url = selectCam(cam_id);
    screenshotIPCam(camera_url);
    
    return 0;
}