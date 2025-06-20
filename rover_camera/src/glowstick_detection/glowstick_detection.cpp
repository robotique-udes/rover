#include "glowstick_detection.hpp"

Glowstick::Glowstick()
{}

bool Glowstick::drawGlowstick(cv::Mat frameGlowstick, const cv::Mat& frameCam)
{

        detector.detectGlowstick(frameCam);

        filterGlowsticks();

        cv::rectangle(frameGlowstick, redGlowstick, cv::Scalar(0, 0, 255), 2);
        cv::rectangle(frameGlowstick, blueGlowstick, cv::Scalar(255, 0, 0), 2);


        return true;
}

bool Glowstick::filterGlowsticks()
{
        uint16_t redLargestArea = 0;
        uint16_t blueLargestArea = 0;

        for (uint16_t i=0;i<detector.redContours.size();i++)
        {
                cv::Rect redRect = cv::boundingRect(detector.redContours[i]);
                if (redRect.area() > redLargestArea)
                {
                        redLargestArea = redRect.area();
                        redGlowstick = redRect;
                }
        }

        for (uint16_t i=0;i<detector.blueContours.size();i++)
        {
                cv::Rect blueRect = cv::boundingRect(detector.blueContours[i]);
                if (blueRect.area() > blueLargestArea)
                {
                        blueLargestArea = blueRect.area();
                        blueGlowstick= blueRect;
                }
        }

        for (uint16_t i=0;i<detector.whiteContours.size();i++)
        {
                cv::Rect whiteGlowstick = cv::boundingRect(detector.whiteContours[i]);
        }

        return true;
}