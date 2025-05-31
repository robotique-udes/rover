#ifndef NAVIGATION_CONTROLLER_HPP
#define NAVIGATION_CONTROLLER_HPP

#include <array>
#include "rover_lib2/helpers/macros.hpp"

class NavigationController
{
    static constexpr float HEADING_BUFFER = 0.0F;

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
        NO_ROTATION = 2
    };

    NavigationController() {};

  private:
    std::array<float, TO_UNDERLYING(eGpsData::eLAST)> _currentGpsData = {0.0F, 0.0F, 0.0F};

    bool _endNodeReached = false;

    double _currentLat;
    double _currentLon;
    double _currentHeading;
    double _targetLat;
    double _targetLon;

  public:
    void getDesiredGpsData(std::array<float, TO_UNDERLYING(eGpsData::eLAST)>& desiredGpsData_)
    {
        _targetLat = desiredGpsData_[TO_UNDERLYING(eGpsData::LATITUDE)];
        _targetLon = desiredGpsData_[TO_UNDERLYING(eGpsData::LONGITUDE)];
    }

    void getCurrentGpsData(std::array<float, TO_UNDERLYING(eGpsData::eLAST)>& currentGpsData_)
    {
        _currentLat = currentGpsData_[TO_UNDERLYING(eGpsData::LATITUDE)];
        _currentLon = currentGpsData_[TO_UNDERLYING(eGpsData::LONGITUDE)];
        _currentHeading = currentGpsData_[TO_UNDERLYING(eGpsData::HEADING)];
    }

    eRotationDirection computeRotationDirection(void)
    {
        float bearing = computeBearing();
        float headingDiff = std::abs(bearing - _currentHeading);

        if (headingDiff <= 180.0F)
        {
            return eRotationDirection::CLOCKWISE;
        }
        else if (headingDiff > 180.0F)
        {
            return eRotationDirection::COUNTERCLOCKWISE;
        }
        else
        {
            return eRotationDirection::NO_ROTATION;
        }
    }

    float computeBearing(void)
    {
        float currentLatRad = _currentLat * M_PI / 180.0F;
        float targetLatRad = _targetLat * M_PI / 180.0F;
        float deltaLonRad = (_targetLon - _currentLon) * M_PI / 180.0F;

        float y = sin(deltaLonRad) * cos(targetLatRad);
        float x = cos(currentLatRad) * sin(targetLatRad) - sin(currentLatRad) * cos(targetLatRad) * cos(deltaLonRad);

        float bearing = atan2(y, x);

        bearing = bearing * 180.0F / M_PI;

        if (bearing < 0.0F)
        {
            bearing += 360.0F;
        }

        return bearing;
    }

};

#endif