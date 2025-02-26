#ifndef __DETECTION_HPP__
#define __DETECTION_HPP__

#include "process_frame.hpp"
#include "rovus_lib/moving_average.hpp"
#include <unordered_map>

#define COEFF_NB_ARUCO 10

class Detection
{
    static constexpr float VALIDATION_THRESHOLD = 0.75;
    static constexpr uint64_t DELAY_BETWEEN_CAPTURE_MS = 50;

  public:
    Detection(std::string cameraURL_);

    std::vector<uint16_t> detect(bool debugMode_);
    void update(bool debugMode_);
    std::vector<uint16_t> getValidatedIds(void);

  private:
    ProcessFrame _processFrame;
    std::unordered_map<uint16_t, MovingAverage<uint16_t, COEFF_NB_ARUCO>> _validation;
    std::vector<uint16_t> _validatedIds;
};

#endif
