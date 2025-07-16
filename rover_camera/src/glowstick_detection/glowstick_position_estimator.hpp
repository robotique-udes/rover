#ifndef GLOWSTICK_POSITION_ESTIMATOR_HPP
#define GLOWSTICK_POSITION_ESTIMATOR_HPP

#include "glowstick.hpp"
#include "image_capture_glowstick.hpp"

class PositionEstimator
{
  public:
    PositionEstimator();
    _Float32 getAngle(const cv::Mat& frame_, Glowstick glowstick_, uint16_t index);
    cv::Point getBotomRectPosition(Glowstick glowstick_, uint16_t index);

  private:
    static constexpr uint16_t FOV = 90;
};

#endif