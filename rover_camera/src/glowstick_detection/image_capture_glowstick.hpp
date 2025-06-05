#ifndef IMAGE_CAPTURE_GLOWSTICK_HPP
#define IMAGE_CAPTURE_GLOWSTICK_HPP

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <optional>
#include <iostream>

class ImageCaptureGlowstick
{
    public:
        ImageCaptureGlowstick(std::string cameraURL_);
        ~ImageCaptureGlowstick(void);

        std::optional<cv::Mat> getFrame(bool debugMode_);
        bool initCam(void);
        bool changeStream(std::string URL_);
        cv::Mat getErrorFrame(void);
        bool isValid(void) const;
        bool isCameraReachable(const std::string& url_, size_t port_, size_t timeoutMs_);
        cv::VideoCapture _cap;

    private:
        bool _isValid;
        std::string _cameraURL;
        bool _firstTryPinningCam = true;

};

#endif