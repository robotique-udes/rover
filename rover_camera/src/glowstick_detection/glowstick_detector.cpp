#include "glowstick_detector.hpp"

GlowstickDetector::GlowstickDetector()
{}

bool GlowstickDetector::detectGlowstick(const cv::Mat& frame)
{
    filterFrame(frame);

    redContours.clear();
    cv::findContours(redMask, redContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    blueContours.clear();
    cv::findContours(blueMask, blueContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    whiteContours.clear();
    cv::findContours(whiteMask, whiteContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    return true;
}

bool GlowstickDetector::filterFrame(const cv::Mat& frame)
{
    cv::Mat hsv;
    cv::Mat red1, red2;

    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> hsvChannels;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    cv::split(hsv, hsvChannels);
    cv::equalizeHist(hsvChannels[2], hsvChannels[2]);
    cv::merge(hsvChannels, hsv);

    cv::inRange(hsv, lowerWhite, upperWhite, whiteMask);

    cv::inRange(hsv, lowerBlue, upperBlue, blueMask);
    cv::dilate(blueMask, blueMask, kernel);

    cv::inRange(hsv, lowerRed1, upperRed1, red1);
    cv::inRange(hsv, lowerRed2, upperRed2, red2);
    
    cv::bitwise_or(red1, red2, redMask);
    cv::dilate(redMask, redMask, kernel);

    cv::GaussianBlur(redMask, redMask, cv::Size(5,5), 0);
    cv::GaussianBlur(whiteMask, whiteMask, cv::Size(5,5), 0);
    cv::GaussianBlur(blueMask, blueMask, cv::Size(5,5), 0);
    

    return true;
}
