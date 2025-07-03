#include "image_capture_glowstick.hpp"


ImageCaptureGlowstick::ImageCaptureGlowstick(std::string cameraURL_) : _cameraURL(cameraURL_)
{
    initCam();
}

ImageCaptureGlowstick::~ImageCaptureGlowstick(void)
{
    _cap.release();
    cv::destroyAllWindows();
}

std::optional<cv::Mat> ImageCaptureGlowstick::getFrame(bool debugMode_)
{
    cv::Mat frame;

    if (!_cap.isOpened() && !changeStream(_cameraURL))
    {
        if (debugMode_)
        {
            frame = getErrorFrame();
            return frame;
        }

        if (_cap.grab())
        {
            _cap.release();
            initCam();
            return std::nullopt;
        }

        _cap.retrieve(frame);
    }

    _firstTryPinningCam = true;

    return frame;

}

bool ImageCaptureGlowstick::changeStream(std::string URL_)
{
    if (URL_ != _cameraURL)
    {
        _cap.release();
        _cameraURL = URL_;
        
        if (!initCam())
        {
            RCLCPP_WARN(rclcpp::get_logger("glowstick_stick_detection_node"), "Could not change streaming device");
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

bool ImageCaptureGlowstick::initCam(void)
{
    bool res;

    res = _cap.open(_cameraURL, cv::CAP_GSTREAMER);

    if (!res)
    {
        RCLCPP_WARN(rclcpp::get_logger("GlowstickDetection"), "Could not open streaming device");
        _isValid = false;
        return false;
    }
    _isValid = true;
    return true;
}

cv::Mat ImageCaptureGlowstick::getErrorFrame(void)
{
    cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
    std::string error_message = "Error: Stream not found!";
    cv::putText(frame, error_message, cv::Point(100, 240), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
    return frame;
}

bool ImageCaptureGlowstick::isValid(void) const
{
    return _isValid;
}

