#ifndef GLOWSTICK_ANGLE_ESTIMATOR_HPP
#define GLOWSTICK_ANGLE_ESTIMATOR_HPP

#include "glowstick.hpp"
#include "image_capture_glowstick.hpp"

class PositionEstimator
{
  public:
    PositionEstimator();
    _Float32 getAngle(const cv::Mat& frame_, Glowstick glowstick_, uint16_t index_);
    cv::Point getBotomRectPosition(Glowstick glowstick_, uint16_t index_);

  private:
    static constexpr uint16_t _FOV = 90;
};

#endif