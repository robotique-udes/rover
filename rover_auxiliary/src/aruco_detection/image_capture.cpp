#include "image_capture.hpp"

ImageCapture::ImageCapture(std::string cameraURL_): _cameraURL(cameraURL_)
{
    initCam();
}

ImageCapture::~ImageCapture(void)
{
    _cap.release();
    cv::destroyAllWindows();
}

bool ImageCapture::initCam()
{
    if (_cap.isOpened())
    {
        return true;
    }

    if (!_cap.open(_cameraURL))
    {
        std::string message = "Could not open streaming device";
        RCLCPP_WARN(rclcpp::get_logger("ArucoDetection"), message.c_str());
        return false;
    }
    return true;
}

bool ImageCapture::manageStream(std::string URL_)
{
    if (URL_ != _cameraURL)
    {
        _cap.release();
        _cameraURL = URL_;

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

std::optional<cv::Mat> ImageCapture::getFrame(bool DEBUG_MODE)
{
    cv::Mat frame;

    if (!_cap.isOpened() && !manageStream(_cameraURL))
    {
        if (DEBUG_MODE)
        {
            getErrorFrame(frame);
            return frame;
        }
        return std::nullopt;
    }

    _cap >> frame;  // Store frame in matrix  (openCV syntax)

    if (frame.empty())
    {
        return std::nullopt;
    }
    return frame;
}

void ImageCapture::getErrorFrame(cv::Mat& frame_)
{
    frame_ = cv::Mat::zeros(480, 640, CV_8UC3);
    std::string error_message = "Error: Stream not found!";
    cv::putText(frame_, error_message, cv::Point(100, 240), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
}
