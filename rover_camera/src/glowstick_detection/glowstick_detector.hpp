#ifndef GLOWSTICK_DETECTOR_HPP
#define GLOWSTICK_DETECTOR_HPP

#include "glowstick.hpp"
#include "glowstick_angle_estimator.hpp"

class GlowstickDetector
{
    enum
    {
        WHITE = 0,
        BLUE = 1,
        RED = 2,
        eLAST
    };

  public:
    GlowstickDetector();
    bool drawGlowsticks(const cv::Mat& frame_, cv::Mat& frameGlowsticks_);
    void detectGlowstick(const cv::Mat& frame_);
    void filterFrame(const cv::Mat& frame_, cv::Mat masks_[]);
    void findGlowsticks(cv::Mat masks_[],
                        std::vector<std::vector<cv::Point>> contours_[],
                        std::vector<std::vector<cv::Point>> contoursWhite_[]);
    void filterGlowsticks(const cv::Mat& frame_, std::vector<std::vector<cv::Point>> contours_[], std::vector<std::vector<cv::Point>> contoursWhite_[]);
    _Float32 glowstickGoal(const cv::Mat& frame_, int color_);

  private:
    Glowstick _glowsticks[eLAST];
    std::vector<cv::Mat> _colorMasks;
    static constexpr uint16_t _maxAmountGlowsticks = GS_CONFIGURATION::MAX_GLOWSTICK_TO_COMPARE;
    PositionEstimator _positionEstimator;
};

#endif