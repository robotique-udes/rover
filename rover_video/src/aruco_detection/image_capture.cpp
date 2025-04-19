#include "image_capture.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

ImageCapture::ImageCapture(std::string cameraURL_):
    _cameraURL(cameraURL_),
    _timer_cameraPinningRetries(DELAY_CAMERA_PINNING_RETRY_MS)
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
    _firstTryPinningCam = true;
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

bool ImageCapture::isCameraReachable(const std::string& url_, int port_, int timeoutMs_)
{
    if (_timer_cameraPinningRetries.isDone() || _firstTryPinningCam)
    {
        _firstTryPinningCam = false;

        size_t start = url_.find("rtsp://");
        if (start == std::string::npos)
            return false;

        start += 7;
        size_t end = url_.find_first_of(":/", start);
        std::string ip = url_.substr(start, end - start);

        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0)
        {
            return false;
        }

        int flags = fcntl(sock, F_GETFL, 0);
        if (flags < 0)
        {
            close(sock);
            return false;
        }

        fcntl(sock, F_SETFL, flags | O_NONBLOCK);

        sockaddr_in addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port_);
        addr.sin_addr.s_addr = inet_addr(ip.c_str());

        int result = connect(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
        if (result < 0 && errno != EINPROGRESS)
        {
            close(sock);
            return false;
        }

        fd_set writefds;
        FD_ZERO(&writefds);
        FD_SET(sock, &writefds);

        struct timeval tv;
        tv.tv_sec = timeoutMs_ / 1000;
        tv.tv_usec = (timeoutMs_ % 1000) * 1000;

        result = select(sock + 1, nullptr, &writefds, nullptr, &tv);
        if (result <= 0)
        {
            close(sock);
            return false;
        }

        int so_error = 0;
        socklen_t len = sizeof(so_error);
        getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &len);

        close(sock);
        return so_error == 0;
    }

    else
    {
        return false;
    }
}
