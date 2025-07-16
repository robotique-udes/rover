#include "glowstick_detector.hpp"

GlowstickDetector::GlowstickDetector()
{
    glowsticks[WHITE] = Glowstick(GS_CONFIGURATION::WHITE::LOWT, GS_CONFIGURATION::WHITE::HIGHT, GS_CONFIGURATION::WHITE::COLOR);
    glowsticks[BLUE] = Glowstick(GS_CONFIGURATION::BLUE::LOWT, GS_CONFIGURATION::BLUE::HIGHT, GS_CONFIGURATION::BLUE::COLOR);
    glowsticks[RED] = Glowstick(GS_CONFIGURATION::RED::LOWT, GS_CONFIGURATION::RED::HIGHT, GS_CONFIGURATION::RED::COLOR);
}

bool GlowstickDetector::drawGlowsticks(const cv::Mat& frame, cv::Mat& frameGlowsticks)
{
    detectGlowstick(frame);

    for (size_t i = 1; i < 3; i++)  // JE VAIS DE 1 A 2 LIVE PCQ LE 1 CEST LE BLANC ET JE VEUX PAS LE VOIR
    {
        for (size_t j = 0; j < maxAmountGlowsticks; j++)
        {
            if (!glowsticks[i]._glowstickRect.empty() && !glowsticks[i]._glowstickRectCenter.empty())
            {
                cv::rectangle(frameGlowsticks, glowsticks[i]._glowstickRect[j], glowsticks[i].getColor(), 2);
                cv::rectangle(frameGlowsticks, glowsticks[i]._glowstickRectCenter[j], GS_CONFIGURATION::WHITE::COLOR, 2);
                std::string angle = std::to_string(_positionEstimator.getAngle(frame, glowsticks[i], j));
                cv::putText(frameGlowsticks,
                            angle,
                            _positionEstimator.getBotomRectPosition(glowsticks[i], j),
                            cv::FONT_HERSHEY_SIMPLEX,
                            1,
                            glowsticks[i].getColor(),
                            4);
            }
        }
    }

    return true;
}

bool GlowstickDetector::detectGlowstick(const cv::Mat& frame)
{
    cv::Mat masks[4];
    std::vector<std::vector<cv::Point>> contours[3];
    std::vector<std::vector<cv::Point>> contoursWhite[3];

    filterFrame(frame, masks);
    findGlowsticks(masks, contours, contoursWhite);
    filterGlowsticks(contours, contoursWhite);

    return true;
}

bool GlowstickDetector::filterFrame(const cv::Mat& frame, cv::Mat masks[])
{
    cv::Mat hsv;
    cv::Mat red1, red2;

    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(20, 20));

    std::vector<cv::Mat> hsvChannels;

    cv::split(hsv, hsvChannels);
    cv::equalizeHist(hsvChannels[2], hsvChannels[2]);
    cv::merge(hsvChannels, hsv);

    for (size_t i = 0; i < 3; i++)
    {
        cv::inRange(hsv, glowsticks[i].getLowerThreshold(), glowsticks[i].getHigherThreshold(), masks[i]);

        if (i == RED)
        {
            cv::inRange(hsv, GS_CONFIGURATION::RED::LOWT2, GS_CONFIGURATION::RED::HIGHT2, masks[i + 1]);
            cv::bitwise_or(masks[i], masks[i + 1], masks[i]);
            masks[i + 1].release();
        }

        if (i == WHITE)
        {
            cv::dilate(masks[i], masks[i], kernel);
        }

        cv::GaussianBlur(masks[i], masks[i], cv::Size(5, 5), 0);
    }

    return true;
}

bool GlowstickDetector::findGlowsticks(cv::Mat masks[],
                                       std::vector<std::vector<cv::Point>> contours[],
                                       std::vector<std::vector<cv::Point>> contoursWhite[])
{
    for (size_t i = 0; i < 3; i++)
    {
        cv::findContours(masks[i], contours[i], cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(masks[0], contoursWhite[i], cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    }

    return true;
}

bool GlowstickDetector::filterGlowsticks(std::vector<std::vector<cv::Point>> contours[],
                                         std::vector<std::vector<cv::Point>> contoursWhite[])
{
    for (size_t i = 0; i < 3; i++)
    {
        std::vector<std::tuple<uint16_t, cv::Rect, cv::Rect>> areaRect;

        for (size_t j = 0; j < contours[i].size(); j++)
        {
            cv::Rect rect = cv::boundingRect(contours[i][j]);

            for (size_t k = 0; k < contoursWhite[i].size(); k++)
            {
                cv::Rect rectCenter = cv::boundingRect(contoursWhite[i][k]);
                uint16_t area = rectCenter.area();

                if ((rect & rectCenter) == rectCenter)
                {
                    areaRect.emplace_back(area, rectCenter, rect);
                }
            }
        }

        std::sort(areaRect.begin(),
                  areaRect.end(),
                  [](const auto& a, const auto& b)
                  {
                      return std::get<0>(a) > std::get<0>(b);
                  });

        glowsticks[i]._glowstickRect.clear();
        glowsticks[i]._glowstickRectCenter.clear();
        size_t count = std::min<size_t>(maxAmountGlowsticks, areaRect.size());

        for (size_t j = 0; j < count; j++)
        {
            glowsticks[i]._glowstickRectCenter.push_back(std::get<1>(areaRect[j]));
            glowsticks[i]._glowstickRect.push_back(std::get<2>(areaRect[j]));
        }
    }

    return true;
}
