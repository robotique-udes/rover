#include "glowstick_detector.hpp"

GlowsitckDetector::GlowsitckDetector()
{}

bool GlowsitckDetector::detectGlowstick(const cv::Mat& frame)
{
    cv::Mat grey, binary;
    std::vector<std::vector<cv::Point>> contours;

    cv::cvtColor(frame, grey, cv::COLOR_BGR2GRAY);
    cv::threshold(grey, binary, brightThreshold, 255, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (uint16_t i=0;i<contours.size();i++)
    {
        const std::vector<cv::Point>& contour = contours[i];
        if (cv::contourArea(contour) > minArea)
        {
            cv::Moments m = cv::moments(contour);
            gsPosition.push_back(cv::Point(m.m10/m.m00, m.m01/m.m00));
        }
    }

    return true;

}

uint16_t GlowsitckDetector::getSize()
{
    return gsPosition.size();
}

std::vector<cv::Point> GlowsitckDetector::getPosition()
{
    return gsPosition;
}
