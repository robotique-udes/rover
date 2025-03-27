#include "detection.hpp"

Detection::Detection(std::string cameraURL_):
    _processFrame(cameraURL_)
{
}

std::vector<uint16_t> Detection::detect(bool debugMode_)
{
    _processFrame.updateDetection(debugMode_);

    if (debugMode_)
    {
        cv::Mat frame = _processFrame.updateDetection(debugMode_).value_or(cv::Mat());
        cv::imshow("Aruco Detection", frame);
        cv::waitKey(30);
    }

    if (!_processFrame.IdsEmpty())
    {
        return _processFrame.getIds();
    }

    return {};
}

void Detection::update(bool debugMode_)
{
    _validatedIds.clear();
    std::vector<uint16_t> detectedIds = detect(debugMode_);

    for (auto it = _validation.begin(); it != _validation.end();)
    {
        uint16_t id = it->first;

        auto found = std::find(detectedIds.begin(), detectedIds.end(), id);

        if (found != detectedIds.end())
        {
            it->second.addValue(static_cast<uint16_t>(1));
            detectedIds.erase(found);

            if (it->second.getAverage() > VALIDATION_THRESHOLD
                && std::find(_validatedIds.begin(), _validatedIds.end(), id) == _validatedIds.end())
            {
                _validatedIds.push_back(id);
            }

            ++it;
        }
        else
        {
            it->second.addValue(static_cast<uint16_t>(0));

            if (it->second.getAverage() == 0)
            {
                it = _validation.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    for (const auto& id : detectedIds)
    {
        _validation.emplace(id, MovingAverage<uint16_t, COEFF_NB_ARUCO>(0));
    }
}

std::vector<uint16_t> Detection::getValidatedIds(void)
{
    return _validatedIds;
}