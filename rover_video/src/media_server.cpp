#include "rclcpp/rclcpp.hpp"
#include "rovus_lib/macros.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

std::string selectCam(int camID)
{
    std::string cam_url = "";

    if(camID == 25)
    {
        cam_url = "rtsp://rover:roverrover@192.168.144.25:554/1/h264major";
    }

    if(camID == 40)
    {
        
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
        // Save the frame as a screenshot
        std::string dir = GET_PACKAGE_SOURCE_DIR("rover_video"); //Comble les variables d'environnement
        std::string filename = std::string(dir) + "/screenshots/ip_camera_screenshot.jpg";
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

    //Penser à créer folder screenshot avant d'en faire



int main()
{
    int cam_id = 0;

    std::cout << "Enter camera ID:" << std::endl;
    std::cin >> cam_id;
    std::string camera_url = selectCam(cam_id);
    screenshotIPCam(camera_url);
    
    return 0;
}