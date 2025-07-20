#include "glowstick_detector.hpp"

GlowstickDetector::GlowstickDetector()
{
    _glowsticks[WHITE] = Glowstick(GS_CONFIGURATION::WHITE::LOW_TH, GS_CONFIGURATION::WHITE::HIGH_TH, GS_CONFIGURATION::WHITE::COLOR);
    _glowsticks[BLUE] = Glowstick(GS_CONFIGURATION::BLUE::LOW_TH, GS_CONFIGURATION::BLUE::HIGH_TH, GS_CONFIGURATION::BLUE::COLOR);
    _glowsticks[RED] = Glowstick(GS_CONFIGURATION::RED::LOW_TH1, GS_CONFIGURATION::RED::HIGH_TH1, GS_CONFIGURATION::RED::COLOR);
}

_Float32 GlowstickDetector::glowstickGoal(const cv::Mat& frame_, int color_)
{
    this->detectGlowstick(frame_);
    return _glowsticks[color_].getAngle();
}

void GlowstickDetector::detectGlowstick(const cv::Mat& frame_)
{
    cv::Mat masks[4];
    std::vector<std::vector<cv::Point>> contours[eLAST];
    std::vector<std::vector<cv::Point>> contoursWhite[eLAST];

    this->filterFrame(frame_, masks);
    this->findGlowsticks(masks, contours, contoursWhite);
    this->filterGlowsticks(frame_, contours, contoursWhite);

}

void GlowstickDetector::filterFrame(const cv::Mat& frame_, cv::Mat masks_[])
{
    cv::Mat frameHueSatVal;

    cv::cvtColor(frame_, frameHueSatVal, cv::COLOR_BGR2HSV);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(20, 20));

    std::vector<cv::Mat> hsvChannels;

    cv::split(frameHueSatVal, hsvChannels);
    cv::equalizeHist(hsvChannels[2], hsvChannels[2]);
    cv::merge(hsvChannels, frameHueSatVal);

    for (size_t i = 0; i < eLAST; i++)
    {
        cv::inRange(frameHueSatVal, _glowsticks[i].getLowerThreshold(), _glowsticks[i].getHigherThreshold(), masks_[i]);

        if (i == RED)
        {
            cv::inRange(frameHueSatVal, GS_CONFIGURATION::RED::LOW_TH2, GS_CONFIGURATION::RED::HIGH_TH2, masks_[i + 1]);
            cv::bitwise_or(masks_[i], masks_[i + 1], masks_[i]);
            masks_[i + 1].release();
        }

        if (i == WHITE)
        {
            cv::dilate(masks_[i], masks_[i], kernel);
        }

        cv::GaussianBlur(masks_[i], masks_[i], cv::Size(5, 5), 0);
    };
}

void GlowstickDetector::findGlowsticks(cv::Mat masks_[],
                                       std::vector<std::vector<cv::Point>> contours_[],
                                       std::vector<std::vector<cv::Point>> contoursWhite_[])
{
    for (size_t i = 1; i < eLAST; i++)
    {
        cv::findContours(masks_[i], contours_[i], cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(masks_[WHITE], contoursWhite_[i], cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    }
}

void GlowstickDetector::filterGlowsticks(const cv::Mat& frame_,
                                         std::vector<std::vector<cv::Point>> contours_[],
                                         std::vector<std::vector<cv::Point>> contoursWhite_[])
{
    for (size_t i = 1; i < eLAST; i++)
    {
        std::vector<std::tuple<uint16_t, cv::Rect, cv::Rect>> areaRect;

        for (size_t j = 0; j < contours_[i].size(); j++)
        {
            cv::Rect rect = cv::boundingRect(contours_[i][j]);

            for (size_t k = 0; k < contoursWhite_[i].size(); k++)
            {
                cv::Rect rectCenter = cv::boundingRect(contoursWhite_[i][k]);
                uint16_t area = rectCenter.area();

                if ((rect & rectCenter) == rectCenter)
                {
                    areaRect.emplace_back(area, rectCenter, rect);
                }

            }
        }

        std::sort(areaRect.begin(),
                  areaRect.end(),
                  [](const auto& areaRectA, const auto& areaRectB)
                  {
                      return std::get<0>(areaRectA) > std::get<0>(areaRectB);
                  });

        _glowsticks[i]._glowstickRect.clear();
        _glowsticks[i]._glowstickRectCenter.clear();
        size_t count = std::min<size_t>(_maxAmountGlowsticks, areaRect.size());

        for (size_t j = 0; j < count; j++)
        {
            if (std::get<0>(areaRect[j]) < GS_CONFIGURATION::MAX_AREA_ACCEPTED)
            {
                _glowsticks[i]._glowstickRectCenter.push_back(std::get<1>(areaRect[j]));
                _glowsticks[i]._glowstickRect.push_back(std::get<2>(areaRect[j]));
                _positionEstimator.calculateAngle(frame_, _glowsticks[i], j);
            }
            else if (count == j+1)
            {
                _glowsticks[i]._glowstickRectCenter.push_back(std::get<1>(areaRect[j]));
                _glowsticks[i]._glowstickRect.push_back(std::get<2>(areaRect[j]));
                _positionEstimator.calculateAngle(frame_, _glowsticks[i], j);
            }
        }
    }
}

bool GlowstickDetector::drawGlowsticks(const cv::Mat& frame_, cv::Mat& frameGlowsticks_)
{
    this->detectGlowstick(frame_);

    bool glowstickDetected[2] = {false, false};
    bool noDetection = true;

    for (size_t i = 0;i<2;i++)
    {
        if (!_glowsticks[i]._glowstickRect.empty() && !_glowsticks[i]._glowstickRectCenter.empty())
        {
            glowstickDetected[i] = true;
            noDetection = false;
        }
    }

    if (noDetection == true) return false;

    for (size_t i = 0; i < 2; i++)  
    {
        if(glowstickDetected[i] == true)
        {

            for (size_t j = 0; j < _maxAmountGlowsticks; j++)
            {
                cv::rectangle(frameGlowsticks_, _glowsticks[i]._glowstickRect[j], _glowsticks[i].getColor(), 2);
                cv::rectangle(frameGlowsticks_, _glowsticks[i]._glowstickRectCenter[j], GS_CONFIGURATION::WHITE::COLOR, 2);
                cv::putText(frameGlowsticks_,
                            std::to_string(_glowsticks[i].getAngle()),
                            _positionEstimator.getBotomRectPosition(_glowsticks[i], j),
                            cv::FONT_HERSHEY_SIMPLEX,
                            1,
                            _glowsticks[i].getColor(),
                            4);
            }

        }
    }

    return true;
}