#ifndef GLOWSTICK_DETECTOR_HPP
#define GLOWSTICK_DETECTOR_HPP

#include "glowstick.hpp"
#include "glowstick_position_estimator.hpp"

class GlowstickDetector 
{
    enum {WHITE, BLUE, RED};

    public:
        GlowstickDetector();
        bool drawGlowsticks(const cv::Mat& frame, cv::Mat& frameGlowsticks);
        bool detectGlowstick(const cv::Mat& frame);
        bool filterFrame(const cv::Mat& frame, cv::Mat masks[]);
        bool findGlowsticks(cv::Mat masks[], std::vector<std::vector<cv::Point>> contours[], std::vector<std::vector<cv::Point>> contoursWhite[]);
        bool filterGlowsticks(std::vector<std::vector<cv::Point>> contours[], std::vector<std::vector<cv::Point>> contoursWhite[]);

    private:
        Glowstick glowsticks[3];
        std::vector<cv::Mat> colorMasks;
        uint16_t maxAmountGlowsticks = 3;
        PositionEstimator _positionEstimator;

};

#endif