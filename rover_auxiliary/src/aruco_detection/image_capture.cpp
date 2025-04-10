#include "image_capture.hpp"

ImageCapture::ImageCapture(std::string cameraURL_):
    _cameraURL(cameraURL_)
{
    _rtspPipeline = "rtspsrc location=" + _cameraURL + PIPELINE;
    initCam();
}

ImageCapture::~ImageCapture(void)
{
    _cap.release();
    cv::destroyAllWindows();
}

bool ImageCapture::initCam(void)
{
    bool res;
    if (_cap.isOpened())
    {
        return true;
    }

    if (_cameraURL.compare(0, 4, "rtsp", 0, 4) == 0)
    {
        res = _cap.open(_rtspPipeline, cv::CAP_GSTREAMER);
    }

    else if (_cameraURL.compare(0, 8, "file:///", 0, 8) == 0)
    {
        _cameraURL = _cameraURL.substr(7);
        res = _cap.open(_cameraURL, cv::CAP_V4L2);
    }

    if (!res)
    {
        RCLCPP_WARN(rclcpp::get_logger("ArucoDetection"), "Could not open streaming device");
        _isValid = false;
        return false;
    }
    _isValid = true;
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
            RCLCPP_WARN(rclcpp::get_logger("aruco_detection_node"), "Could not change streaming device");
            _isValid = false;
            return false;
        }
        _isValid = true;
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

    if (!_cap.grab())
    {
        _cap.release();
        initCam();
        return std::nullopt;
    }

    _cap.retrieve(frame);

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

bool ImageCapture::isValid(void) const
{
    return _isValid;
}
