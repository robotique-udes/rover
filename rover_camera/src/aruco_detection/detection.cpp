#include "detection.hpp"

Detection::Detection(const std::string& cameraURL_, uint8_t detectionTag_):
    _processFrame(cameraURL_)
{
    _tag = detectionTag_;
    _cameraURL = cameraURL_;
}

std::optional<std::vector<uint16_t>> Detection::detect(bool debugMode_)
{
    if (!(_processFrame.updateDetection(debugMode_).has_value()))
    {
        return std::nullopt;
    }

    if (debugMode_)
    {
        cv::Mat frame = _processFrame.updateDetection(debugMode_).value_or(cv::Mat());
        cv::imshow("Aruco Detection" + std::to_string(_tag), frame);
        cv::waitKey(30);
    }

    if (!_processFrame.IdsEmpty())
    {
        return _processFrame.getIds();
    }

    return std::vector<uint16_t>{};
}

void Detection::update(bool debugMode_)
{
    _validatedIds.clear();
    std::optional<std::vector<uint16_t>> detectionResult = this->detect(debugMode_);
    this->setCamLost(!detectionResult.has_value());

    if (this->getCamLost())
    {
        return;
    }

    std::vector<uint16_t> detectedIds = detectionResult.value();
    for (auto it = _validation.begin(); it != _validation.end();)
    {
        uint16_t id = it->first;

        auto found = std::ranges::find(detectedIds, id);

        if (found != detectedIds.end())
        {
            it->second.addValue(static_cast<uint16_t>(1));
            detectedIds.erase(found);

            if (it->second.getFilteredValue() > VALIDATION_THRESHOLD && std::ranges::find(_validatedIds, id) == _validatedIds.end())
            {
                _validatedIds.push_back(id);
            }

            ++it;
        }
        else
        {
            it->second.addValue(static_cast<uint16_t>(0));

            if (it->second.getFilteredValue() == 0)
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
        _validation.try_emplace(id, 0);
    }
}

std::vector<uint16_t> Detection::getValidatedIds(void) const
{
    return _validatedIds;
}

std::string Detection::getCamURL(void) const
{
    return _cameraURL;
}

uint8_t Detection::getTag(void) const
{
    return _tag;
}

bool Detection::isValid(void) const
{
    return _processFrame.isValid();
}

bool Detection::getCamLost(void) const
{
    return _camLost;
}

void Detection::setCamLost(bool camLost_)
{
    _camLost = camLost_;
}
