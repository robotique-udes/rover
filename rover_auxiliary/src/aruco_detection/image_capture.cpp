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

bool ImageCapture::initCam(void)
{
    if (_cap.isOpened())
    {
        return true;
    }

    _pipeline = "rtspsrc location=" + _cameraURL
                + " latency=0 drop=true ! decodebin ! videoconvert ! queue max-size-buffers=1 ! appsink";
    if (!_cap.open(_pipeline, cv::CAP_GSTREAMER))
    {
        RCLCPP_WARN(rclcpp::get_logger("ArucoDetection"), "Could not open streaming device");
        return false;
    }
    return true;
}

bool ImageCapture::changeStream(std::string URL_)
{
    if (URL_ != _cameraURL)
    {
        _cap.release();
        _cameraURL = URL_;

        if (!initCam())
        {
            RCLCPP_WARN(rclcpp::get_logger("ArucoDetection"), "Could not change streaming device");
            return false;
        }
        return true;
    }
    else
    {
        return initCam();
    }
}

std::optional<cv::Mat> ImageCapture::getFrame(bool debugMode_)
{
    cv::Mat frame;

    if (!_cap.isOpened() && !changeStream(_cameraURL))
    {
        if (debugMode_)
        {
            frame = getErrorFrame();
            return frame;
        }
        return std::nullopt;
    }

    _cap.read(frame);

    if (frame.empty())
    {
        return std::nullopt;
    }
    return frame;
}

cv::Mat ImageCapture::getErrorFrame(void)
{
    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
    std::string error_message = "Error: Stream not found!";
    cv::putText(frame, error_message, cv::Point(100, 240), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
    return frame;
}