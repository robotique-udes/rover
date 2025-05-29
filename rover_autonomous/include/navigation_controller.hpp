#ifndef NAVIGATION_CONTROLLER_HPP
#define NAVIGATION_CONTROLLER_HPP

#include <array>
#include "rover_lib2/helpers/macros.hpp"

class NavigationController
{
    static constexpr float HEADING_BUFFER = 1.0F;

  public:
    enum class eGpsData
    {
        LATITUDE = 0,
        LONGITUDE = 1,
        HEADING = 2,
        eLAST
    };

    enum class eRotationDirection
    {
        CLOCKWISE = 0,
        COUNTERCLOCKWISE = 1,
        eLAST
    };

    NavigationController() {};

  private:
    std::array<float, TO_UNDERLYING(eGpsData::eLAST)> _currentGpsData = {0.0F, 0.0F, 0.0F};
    std::array<float, TO_UNDERLYING(eGpsData::eLAST)> _desiredGpsData = {0.0F, 0.0F, 0.0F};

    bool _endNodeReached = false;

  public:
    void getDesiredGpsData(std::array<float, TO_UNDERLYING(eGpsData::eLAST)>& desiredGpsData_)
    {
        desiredGpsData_ = _desiredGpsData;
    }

    void getCurrentGpsData(std::array<float, TO_UNDERLYING(eGpsData::eLAST)>& currentGpsData_)
    {
        currentGpsData_ = _currentGpsData;
    }

    eRotationDirection setDesiredRotation(void)
    {
        float headingDiff = _desiredGpsData[TO_UNDERLYING(eGpsData::HEADING)] - _currentGpsData[TO_UNDERLYING(eGpsData::HEADING)];

        if (headingDiff <= HEADING_BUFFER)
        {
            return;
        }

        if (std::abs(headingDiff) <= 180.0F)
        {
            return eRotationDirection::CLOCKWISE;
        }
        else
        {
            return eRotationDirection::COUNTERCLOCKWISE;
        }
    }
};

#endif