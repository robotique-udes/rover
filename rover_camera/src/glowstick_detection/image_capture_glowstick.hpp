#ifndef IMAGE_CAPTURE_GLOWSTICK_HPP
#define IMAGE_CAPTURE_GLOWSTICK_HPP

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>

class ImageCaptureGlowstick
{
    public:
        ImageCaptureGlowstick(std::string cameraURL_);
        ~ImageCaptureGlowstick(void);
        bool initCam(void);

    private:
        bool isValid;
        std::string _cameraURL;
        cv::VideoCapture _cap;

}

#endif