#ifndef __DETECTION_HPP__
#define __DETECTION_HPP__

#include "frame_processing.hpp"
#include "rovus_lib/moving_average.hpp"
//#include "rovus_lib/timer.hpp"
#include <unordered_map>

#define COEFF_NB_ARUCO 10

class ArucoDetection
{
  public:
    ArucoDetection(std::string _cameraURL);
    ~ArucoDetection(void);
    std::vector<uint16_t> detect(bool DEBUG_MODE);
    std::vector<uint16_t> update(bool DEBUG_MODE);

  private:

    static constexpr float VALIDATION_THRESHOLD = 0.75;
    static constexpr uint8_t DELAY_BETWEEN_CAPTURE_MS = 100;
    FrameProcessing processedFrame;
    std::unordered_map<uint16_t, MovingAverage<uint16_t, COEFF_NB_ARUCO>> validation;
};

#endif