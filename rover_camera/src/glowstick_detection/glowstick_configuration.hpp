#ifndef GLOWSTICK_CONFIGURATION_HPP
#define GLOWSTICK_CONFIGURATION_HPP

#include <opencv2/opencv.hpp>

namespace GS_CONFIGURATION
{
    namespace WHITE
    {
        const cv::Scalar LOWT(0, 0, 200);
        const cv::Scalar HIGHT(180, 180, 255);
        const cv::Scalar COLOR(0, 255, 0);
    }

    namespace BLUE
    {
        const cv::Scalar LOWT(100, 150, 150);
        const cv::Scalar HIGHT(130, 255, 255);
        const cv::Scalar COLOR(255, 0, 0);
    }

    namespace RED
    {
        const cv::Scalar LOWT(0, 150, 150);
        const cv::Scalar HIGHT(10, 255, 255);
        const cv::Scalar COLOR(0, 0, 255);

        const cv::Scalar LOWT2(170, 150, 150);
        const cv::Scalar HIGHT2(180, 255, 255);
    }

}

#endif