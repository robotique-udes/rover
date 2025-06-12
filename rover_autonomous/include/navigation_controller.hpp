#ifndef NAVIGATION_CONTROLLER_HPP
#define NAVIGATION_CONTROLLER_HPP

#include <array>
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/constants.hpp"

class NavigationController
{
    // static constexpr float HEADING_BUFFER = 1.0F;
    static constexpr float POSITION_BUFFER = 1.0F;
    static constexpr float RECTIFICATION_FACTOR = 1.2F;
    static constexpr float EARTH_RADIUS_METERS = 6'378'137.0F;  // Radius of the Earth in meters

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

    enum class eWheelCmd
    {
        FRONT_LEFT = 0,
        REAR_LEFT = 1,
        FRONT_RIGHT = 2,
        REAR_RIGHT = 3,
        eLAST
    };

    NavigationController() {};

    float headingBuffer_;
    bool _desiredHeadingReached = false;

  private:
    std::array<float, TO_UNDERLYING(eGpsData::eLAST)> _currentGpsData = {0.0F, 0.0F, 0.0F};

    bool _endNodeReached = false;

    double _currentLat;
    double _currentLon;
    double _currentHeading;
    double _targetLat;
    double _targetLon;

    std::array<float, TO_UNDERLYING(eWheelCmd::eLAST)> _targetWheelCmd;

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

        if (headingDiff <= this->headingBuffer_)
        {
            return eRotationDirection::NO_ROTATION;
        }
        if (headingDiff > this->headingBuffer_ && headingDiff <= 180.0F)
        {
            return eRotationDirection::CLOCKWISE;
        }
        else if (headingDiff > this->headingBuffer_ && headingDiff > 180.0F)
        {
            return eRotationDirection::COUNTERCLOCKWISE;
        }
    }

    std::array<float, TO_UNDERLYING(eWheelCmd::eLAST)> idleCmd(void)
    {
        _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_LEFT)] = 5.0F;
        _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_LEFT)] = 5.0F;
        _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_RIGHT)] = 5.0F;
        _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_RIGHT)] = 5.0F;

        return _targetWheelCmd;
    }

    std::array<float, TO_UNDERLYING(eWheelCmd::eLAST)> getToHeading(void)
    {
        eRotationDirection rotationDirection = this->computeRotationDirection();

        if (rotationDirection == eRotationDirection::CLOCKWISE)
        {
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_LEFT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_LEFT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_RIGHT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL * -1.0F;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_RIGHT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL * -1.0F;
        }
        else if (rotationDirection == eRotationDirection::COUNTERCLOCKWISE)
        {
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_LEFT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL * -1.0F;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_LEFT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL * -1.0F;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_RIGHT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_RIGHT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
        }
        else
        {
            _desiredHeadingReached = true;
        }

        return _targetWheelCmd;
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

    float getDistance(void)
    {
        float lat1Rad = _currentLat * M_PI / 180.0F;
        float lat2Rad = _targetLat * M_PI / 180.0F;
        float deltaLatRad = (lat2Rad - lat1Rad);
        float deltaLonRad = (_targetLon - _currentLon) * M_PI / 180.0F;

        float a = sin(deltaLatRad / 2.0F) * sin(deltaLatRad / 2.0F)
                  + cos(lat1Rad) * cos(lat2Rad) * sin(deltaLonRad / 2.0F) * sin(deltaLonRad / 2.0F);
        float c = 2.0F * atan2(sqrt(a), sqrt(1 - a));

        float distance = EARTH_RADIUS_METERS * c;

        return distance;
    }

    std::array<float, TO_UNDERLYING(eWheelCmd::eLAST)> setWheelCmd(void)
    {
        if (this->getDistance() <= POSITION_BUFFER)
        {
            _endNodeReached = true;

            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_LEFT)] = 0.0F;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_LEFT)] = 0.0F;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_RIGHT)] = 0.0F;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_RIGHT)] = 0.0F;
            return _targetWheelCmd;
        }
        else
        {
            if (this->computeRotationDirection() == eRotationDirection::CLOCKWISE)
            {
                _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_LEFT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
                _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_LEFT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
                _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_RIGHT)]
                    = Constants::DriveTrain::SPEED_FACTOR_NORMAL * RECTIFICATION_FACTOR;
                _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_RIGHT)]
                    = Constants::DriveTrain::SPEED_FACTOR_NORMAL * RECTIFICATION_FACTOR;
            }
            else if (this->computeRotationDirection() == eRotationDirection::COUNTERCLOCKWISE)
            {
                _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_LEFT)]
                    = Constants::DriveTrain::SPEED_FACTOR_NORMAL * RECTIFICATION_FACTOR;
                _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_LEFT)]
                    = Constants::DriveTrain::SPEED_FACTOR_NORMAL * RECTIFICATION_FACTOR;
                _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_RIGHT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
                _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_RIGHT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
            }
            else
            {
                _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_LEFT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
                _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_LEFT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
                _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_RIGHT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
                _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_RIGHT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
            }
        }

        return _targetWheelCmd;
    }
};

#endif