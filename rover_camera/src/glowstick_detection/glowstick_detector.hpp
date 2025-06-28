#ifndef GLOWSTICK_DETECTOR_HPP
#define GLOWSTICK_DETECTOR_HPP

#include "glowstick.hpp"

class GlowstickDetector 
{
    enum {WHITE, BLUE, RED};

    public:
        GlowstickDetector();
        bool drawGlowsticks(const cv::Mat& frame, cv::Mat& frameGlowsticks);
        bool detectGlowstick(const cv::Mat& frame);
        bool filterFrame(const cv::Mat& frame, cv::Mat masks[]);
        bool findGlowsticks(cv::Mat masks[], std::vector<std::vector<cv::Point>> contours[]);
        bool filterGlowsticks(std::vector<std::vector<cv::Point>> contours[]);
        std::vector<cv::Rect> confirmedGlowsticks[3];

    private:
        Glowstick glowsticks[3];
        std::vector<cv::Mat> colorMasks;
        uint16_t maxAmountGlowsticks = 3;


};

#endif