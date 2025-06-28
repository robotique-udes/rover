#include "glowstick_detector.hpp"

GlowstickDetector::GlowstickDetector()
{
    glowsticks[WHITE] = Glowstick(cv::Scalar(0, 0, 240), cv::Scalar(180, 30, 255), cv::Scalar(0,255,0));
    glowsticks[BLUE] = Glowstick(cv::Scalar(100, 150, 150), cv::Scalar(130, 255, 255), cv::Scalar(255,0,0)); 
    glowsticks[RED] = Glowstick(cv::Scalar(0, 150, 150), cv::Scalar(10, 255, 255), cv::Scalar(0,0,255));
}

bool GlowstickDetector::drawGlowsticks(const cv::Mat& frame, cv::Mat& frameGlowsticks)
{

        detectGlowstick(frame);

        for (uint16_t i=0;i<3;i++)
        {
                if (!confirmedGlowsticks[i].empty())
                {
                        cv::rectangle(frameGlowsticks, confirmedGlowsticks[i][0], glowsticks[i].getColor(), 2);  
                }
                
        }

        return true;
}

bool GlowstickDetector::detectGlowstick(const cv::Mat& frame)
{
    cv::Mat masks[4];
    std::vector<std::vector<cv::Point>> contours[3];

    filterFrame(frame, masks);
    findGlowsticks(masks, contours);
    filterGlowsticks(contours);

    return true;
}

bool GlowstickDetector::filterFrame(const cv::Mat& frame, cv::Mat masks[])
{
    cv::Mat hsv;
    cv::Mat red1, red2;

    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> hsvChannels;
    //cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    cv::split(hsv, hsvChannels);
    cv::equalizeHist(hsvChannels[2], hsvChannels[2]);
    cv::merge(hsvChannels, hsv);

    for (uint16_t i=0;i<3;i++)
    {
        cv::inRange(hsv, glowsticks[i].getLowerThreshold(), glowsticks[i].getHigherThreshold(), masks[i]);

        if (i==RED)
        {
                cv::inRange(hsv, cv::Scalar(170, 150, 150), cv::Scalar(180, 255, 255), masks[i+1]);
                cv::bitwise_or(masks[i], masks[i+1], masks[i]);
                masks[i+1].release();
        }

        cv::GaussianBlur(masks[i], masks[i], cv::Size(5,5), 0);
    }
    //cv::dilate(blueMask, blueMask, kernel);
    //cv::dilate(redMask, redMask, kernel);
    
    return true;
}

bool GlowstickDetector::findGlowsticks(cv::Mat masks[], std::vector<std::vector<cv::Point>> contours[])
{
        for (uint16_t i=0;i<3;i++)
        {
                cv::findContours(masks[i], contours[i], cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        }

        return true;
}

bool GlowstickDetector::filterGlowsticks(std::vector<std::vector<cv::Point>> contours[])
{
        uint16_t largestArea[3] = {0,0,0};

        for (uint16_t i=0;i<3;i++)
        {
                for (uint16_t j=0;j<contours[i].size();j++)
                {
                        cv::Rect rect = cv::boundingRect(contours[i][j]);
                        if (rect.area() > largestArea[i])
                        {
                                largestArea[i] = rect.area();
                                confirmedGlowsticks[i].clear();
                                confirmedGlowsticks[i].push_back(rect);
                        }
                }
        }

        return true;
}