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

std::optional<cv::Mat> ImageCaptureGlowstick::getFrame(double debugMode_)
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

    if (frame.empty())
    {
        return std::nullopt;
    }

}


void ImageCaptureGlowstick::ifinitCam(void)
{
        if (!_cap.isOpened())
    {
        return true;
    }

    if (_cameraURL.compare(0, 4, "rtsp", 0, 4) == 0)
    {
        if (isCameraReachable(_cameraURL, CAM_NETWORK_PORT, TIMEOUT_CAMERA_PINNING__MS))
            res = _cap.open(_rtspPipeline, cv::CAP_GSTREAMER);
        else
        {
            res = false;
        }
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

bool ImageCaptureGlowstick::isCameraReachable(const std::string& url_, size_t port_, size_t timeoutMs_)
{
    if (!_timer_cameraPinningRetries.isReady() && !_firstTryPinningCam)
    {
        return false;
    }
    _firstTryPinningCam = false;

    bool res = RoverLib2::isIPReachable(url_, port_, timeoutMs_);
    return res;
}




int main()
{
    rclcpp::init(argc, argv);

    ImageCaptureGlowstick webcam("/dev/video0");

    frame = webcam.getFrame();

    while (true)
    {
        cap >> frame;


        cv::imshow("Laptopframe", frame);

        if (cv::waitKey(1) == 27)
        {
            break;
        }
    }

}
