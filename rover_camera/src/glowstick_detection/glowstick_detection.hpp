#ifndef GLOWSTICK_DETECTION_HPP
#define GLOWSTICK_DETECTION_HPP

#include "glowstick_detector.hpp"

class Glowstick
{
    public:
        Glowstick();
        bool drawGlowstick(cv::Mat frameGlowstick, const cv::Mat& frameCam);
    private:
        GlowstickDetector detector;
};


#endif