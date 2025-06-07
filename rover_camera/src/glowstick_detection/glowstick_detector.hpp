#ifndef GLOWSTICK_DETECTOR_HPP
#define GLOWSTICK_DETECTOR_HPP

#include "image_capture_glowstick.hpp"
#include <vector>

class GlowsitckDetector 
{
    enum Color {RED, GREEN, BLUE, WHITE};


    public:
        GlowsitckDetector();
        bool detectGlowstick(const cv::Mat& frame);
        uint16_t getSize();
        std::vector<cv::Point> getPosition();

    private:
        const uint16_t brightThreshold = 220;
        const uint16_t minArea = 10;
        std::vector<cv::Point> gsPosition;


};

#endif