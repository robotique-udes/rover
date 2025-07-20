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
    _Float32 getAngle(void);
    void setAngle(_Float32 angle_);
    std::vector<cv::Rect> _glowstickRect;
    std::vector<cv::Rect> _glowstickRectCenter;


  private:
    cv::Scalar _lowThreshold;
    cv::Scalar _highThreshold;
    cv::Scalar _color;
    _Float32 _angle = 0;
};

#endif