#include "glowstick.hpp"

Glowstick::Glowstick() {}

Glowstick::Glowstick(cv::Scalar low_, cv::Scalar high_, cv::Scalar color_)
{
    _lowThreshold = low_;
    _highThreshold = high_;
    _color = color_;
}

cv::Scalar Glowstick::getColor(void)
{
    return _color;
}

cv::Scalar Glowstick::getLowerThreshold(void)
{
    return _lowThreshold;
}

cv::Scalar Glowstick::getHigherThreshold(void)
{
    return _highThreshold;
}