#ifndef __DETECTION_HPP__
#define __DETECTION_HPP__

#include "process_frame.hpp"
#include "rovus_lib/moving_average.hpp"
#include <unordered_map>

#define COEFF_NB_ARUCO 10

class Detection
{
    static constexpr float VALIDATION_THRESHOLD = 0.75;

  public:
    Detection(std::string cameraURL_, uint8_t detectionTag_);

    std::optional<std::vector<uint16_t>> detect(bool debugMode_);
    void update(bool debugMode_);
    std::vector<uint16_t> getValidatedIds(void) const;
    std::string getCamURL(void) const;
    uint8_t getTag(void) const;
    bool isValid(void) const;
    bool camLost(void) const;

  private:
    uint8_t _tag;
    bool _camLost = false;
    std::string _cameraURL;
    ProcessFrame _processFrame;
    std::unordered_map<uint16_t, MovingAverage<uint16_t, COEFF_NB_ARUCO>> _validation;
    std::vector<uint16_t> _validatedIds;
};

#endif
