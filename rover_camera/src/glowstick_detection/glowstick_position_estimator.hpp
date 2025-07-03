#ifndef GLOWSTICK_POSITION_ESTIMATOR_HPP
#define GLOWSTICK_POSITION_ESTIMATOR_HPP

#include "glowstick_detector.hpp"

class PositionEstimator
{

    public:
        PositionEstimator();

    private:
        GlowstickDetector detectedGlowsticks;

};

#endif