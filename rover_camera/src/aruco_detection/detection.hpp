#ifndef DETECTION_HPP
#define DETECTION_HPP

#include "process_frame.hpp"

#include <rover_lib2/helpers/moving_average.hpp>
#include <unordered_map>

class Detection
{
    static constexpr uint16_t COEFF_NB_ARUCO = 10U;
    static constexpr float VALIDATION_THRESHOLD = 0.75F;

  public:
    Detection(const std::string& cameraURL_, uint8_t detectionTag_);

    std::optional<std::vector<uint16_t>> detect(bool debugMode_);
    void update(bool debugMode_);
    std::vector<uint16_t> getValidatedIds(void) const;
    std::string getCamURL(void) const;
    uint8_t getTag(void) const;
    bool isValid(void) const;
    bool getCamLost(void) const;
    void setCamLost(bool camLost_);

  private:
    uint8_t _tag;
    bool _camLost = false;
    std::string _cameraURL;
    ProcessFrame _processFrame;
    std::unordered_map<uint16_t, MovingAverage<uint16_t, COEFF_NB_ARUCO>> _validation;
    std::vector<uint16_t> _validatedIds;
};

#endif
