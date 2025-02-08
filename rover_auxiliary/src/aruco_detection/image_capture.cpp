#include "image_capture.hpp"

ImageCapture::ImageCapture(std::string _cameraURL): cameraURL(_cameraURL)
{
    initCam();
}

ImageCapture::~ImageCapture(void)
{
    cap.release();
    cv::destroyAllWindows();
}

bool ImageCapture::initCam()
{
    if (!cap.isOpened())
    {
        cap.open(cameraURL);
    }

    else
        return true;

    if (!cap.isOpened())
    {
        std::string message = "Could not open streaming device";
        RCLCPP_WARN(rclcpp::get_logger("ArucoDetection"), message.c_str());
        return false;
    }
    return true;
}

bool ImageCapture::manageStream(std::string URL)
{
    if (URL != cameraURL)
    {
        cap.release();
        cameraURL = URL;

        if (!initCam())
        {
            std::string message = "Could not change streaming device";
            RCLCPP_WARN(rclcpp::get_logger("ArucoDetection"), message.c_str());
            return false;
        }
        return true;
    }
    else
    {
        return initCam();
    }
}

cv::Mat ImageCapture::getFrame(bool DEBUG_MODE)
{
    cv::Mat frame;

    if (!cap.isOpened())
    {
        if (!manageStream(cameraURL))
        {  // Open camera only if it's off
            if (DEBUG_MODE)
            {
                getErrorFrame(frame);
            }

            return frame;
        }
    }

    cap >> frame;  // Store frame in matrix  (openCV syntax)
    return frame;
}

void ImageCapture::getErrorFrame(cv::Mat& frame)
{
    frame = cv::Mat::zeros(480, 640, CV_8UC3);
    std::string error_message = "Error: Stream not found!";
    cv::putText(frame, error_message, cv::Point(100, 240), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
}
