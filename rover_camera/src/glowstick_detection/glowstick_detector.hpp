#ifndef GLOWSTICK_DETECTOR_HPP
#define GLOWSTICK_DETECTOR_HPP

#include "image_capture_glowstick.hpp"

class GlowstickDetector 
{
    public:
        GlowstickDetector();
        bool detectGlowstick(const cv::Mat& frame);
        bool filterFrame(const cv::Mat& frame);
        std::vector<std::vector<cv::Point>> redContours;
        std::vector<std::vector<cv::Point>> blueContours;
        std::vector<std::vector<cv::Point>> whiteContours;

    private:
        cv::Scalar lowerBlue = cv::Scalar(100, 150, 50);
        cv::Scalar upperBlue = cv::Scalar(130, 255, 255);
        cv::Scalar lowerRed1 = cv::Scalar(0, 180, 100);
        cv::Scalar upperRed1 = cv::Scalar(10, 255, 255);
        cv::Scalar lowerRed2 = cv::Scalar(170, 180, 100);
        cv::Scalar upperRed2 = cv::Scalar(180, 255, 255);
        cv::Scalar lowerWhite = cv::Scalar(0, 0, 240);
        cv::Scalar upperWhite = cv::Scalar(180, 30, 255);
        cv::Mat whiteMask;
        cv::Mat blueMask;
        cv::Mat redMask;


};

#endif