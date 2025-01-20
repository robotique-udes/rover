#include "rclcpp/rclcpp.hpp"
#include "rovus_lib/macros.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"



int screenshotIPCam() {
    // The URL format will depend on the camera model and configuration
    std::string camera_url = "rtsp://rover:roverrover@192.168.144.25:554/1/h264major";

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
        std::string filename = "~/screenshots/ip_camera_screenshot.jpg";
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

int videoIPCam()
{
    return 1;
}