#ifndef GLOWSTICK_CONFIGURATION_HPP
#define GLOWSTICK_CONFIGURATION_HPP

#include <opencv2/opencv.hpp>

namespace GS_CONFIGURATION
{
    namespace WHITE
    {
        const cv::Scalar LOW_TH(0, 0, 200);
        const cv::Scalar HIGH_TH(180, 180, 255);
        const cv::Scalar COLOR(0, 255, 0);
    }

    namespace BLUE
    {
        const cv::Scalar LOW_TH(100, 150, 150);
        const cv::Scalar HIGH_TH(130, 255, 255);
        const cv::Scalar COLOR(255, 0, 0);
    }

    namespace RED
    {
        const cv::Scalar LOW_TH1(0, 150, 150);
        const cv::Scalar HIGH_TH1(10, 255, 255);
        const cv::Scalar COLOR(0, 0, 255);

        const cv::Scalar LOW_TH2(170, 150, 150);
        const cv::Scalar HIGH_TH2(180, 255, 255);
    }

    constexpr u_int16_t WAIT_KEY_DELAY_MS = 20;

}

#endif