#ifndef GLOWSTICK_HPP
#define GLOWSTICK_HPP

#include "image_capture_glowstick.hpp"

class Glowstick
{
    public:
        Glowstick();
        Glowstick(cv::Scalar low_, cv::Scalar high_, cv::Scalar color_);
        cv::Scalar getColor(void);
        cv::Scalar getLowerThreshold(void);
        cv::Scalar getHigherThreshold(void);
        std::vector<cv::Rect> _glowstickRect;
        std::vector<cv::Rect> _glowstickRectCenter;
    private:
        cv::Scalar _lowThreshold;
        cv::Scalar _highThreshold;
        cv::Scalar _color;
};


#endif