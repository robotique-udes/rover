#include "glowstick_angle_estimator.hpp"

PositionEstimator::PositionEstimator() {}

_Float32 PositionEstimator::getAngle(const cv::Mat& frame_, Glowstick glowstick_, uint16_t index_)
{
    _Float32 frameWidth = frame_.cols;
    _Float32 frameMidlleX = frame_.cols / 2;
    cv::Point glowstickCenter(glowstick_._glowstickRectCenter[index_].x + glowstick_._glowstickRectCenter[index_].width / 2,
                              glowstick_._glowstickRectCenter[index_].y + glowstick_._glowstickRectCenter[index_].height / 2);

    _Float32 dx = glowstickCenter.x - frameMidlleX;

    _Float32 FOVRad = _FOV * CV_PI / 180.0;

    _Float32 focalLength = (frameMidlleX) / tan(FOVRad / 2.0);

    _Float32 pixelsPerDegree = frameWidth / _FOV;

    _Float32 angleRad = atan(dx / focalLength);

    return angleRad * 180.0 / CV_PI;
}

cv::Point PositionEstimator::getBotomRectPosition(Glowstick glowstick_, uint16_t index_)
{
    return cv::Point(glowstick_._glowstickRectCenter[index_].x,
                     glowstick_._glowstickRectCenter[index_].y + glowstick_._glowstickRectCenter[index_].height + 20);
}
