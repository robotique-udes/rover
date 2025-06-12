#include "glowstick_detection.hpp"

Glowstick::Glowstick()
{}

bool Glowstick::drawGlowstick(cv::Mat frameGlowstick, const cv::Mat& frameCam)
{

        detector.detectGlowstick(frameCam);
        
        cv::drawContours(frameGlowstick, detector.redContours, -1, cv::Scalar(0, 0, 255),2);
        cv::drawContours(frameGlowstick, detector.blueContours, -1, cv::Scalar(255, 0, 0),2);
        cv::drawContours(frameGlowstick, detector.whiteContours, -1, cv::Scalar(0, 255, 0),2);

        return true;
}