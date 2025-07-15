#ifndef NAVIGATION_CONTROLLER_HPP
#define NAVIGATION_CONTROLLER_HPP

#include <array>
#include <queue>
#include <cmath>
#include <algorithm>
#include <deque>
#include <vector>
#include <numbers>

#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include "lidar_config.hpp"

class NavigationController
{
    static constexpr float HEADING_BUFFER =  20.0F;
    static constexpr float POSITION_BUFFER = 1.0F;
    static constexpr float RECTIFICATION_FACTOR = 0.8F;
    static constexpr float EARTH_RADIUS_METERS = 6'378'137.0F;

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

    enum class eForceVector
    {
        FORCE_X = 0,
        FORCE_Y = 1,
        eLAST
    };

    enum class eTotalForce
    {
        MAGNITUDE = 0,
        YAW = 1,
        eLAST
    };

    NavigationController() {};

    float headingBuffer_ = HEADING_BUFFER;
    bool _desiredHeadingReached = false;
    bool _endNodeReached = false;
    double _currentLat = 0.0;
    double _currentLon = 0.0;
    double _currentHeading = 0.0;
    double _targetLat = 0.0;
    double _targetLon = 0.0;

  private:
    std::array<float, TO_UNDERLYING(eGpsData::eLAST)> _currentGpsData;
    std::array<float, TO_UNDERLYING(eWheelCmd::eLAST)> _targetWheelCmd;

  public:
    void getDesiredGpsData(std::array<double, TO_UNDERLYING(eGpsData::eLAST)>& desiredGpsData_)
    {
        _targetLat = desiredGpsData_[TO_UNDERLYING(eGpsData::LATITUDE)];
        _targetLon = desiredGpsData_[TO_UNDERLYING(eGpsData::LONGITUDE)];
    }

    void getCurrentGpsData(std::array<double, TO_UNDERLYING(eGpsData::eLAST)>& currentGpsData_)
    {
        _currentLat = currentGpsData_[TO_UNDERLYING(eGpsData::LATITUDE)];
        _currentLon = currentGpsData_[TO_UNDERLYING(eGpsData::LONGITUDE)];
        _currentHeading = currentGpsData_[TO_UNDERLYING(eGpsData::HEADING)];
    }

    eRotationDirection computeRotationDirection(double headingDiff_)
    {
        printf("Heading diff: %.2f degrees\n", headingDiff_);
        printf("Headind buffer: %.2f degrees\n", this->headingBuffer_);
        if (headingDiff_ <= this->headingBuffer_)
        {
            printf("No rotation needed \n");
            this->_desiredHeadingReached = true;
            return eRotationDirection::NO_ROTATION;
        }
        if (headingDiff_ > this->headingBuffer_ && headingDiff_ <= 180.0F)
        {
            printf("Clockwise \n");
            return eRotationDirection::CLOCKWISE;
        }
        else if (headingDiff_ > this->headingBuffer_ && headingDiff_ > 180.0F)
        {
            printf("Counterclockwise \n");
            return eRotationDirection::COUNTERCLOCKWISE;
        }
    }

    std::array<float, TO_UNDERLYING(eWheelCmd::eLAST)> navigate(std::array<float, TO_UNDERLYING(eTotalForce::eLAST)> totalForces_)
    {
        float yaw = totalForces_[TO_UNDERLYING(eTotalForce::YAW)];
        float magnitude = totalForces_[TO_UNDERLYING(eTotalForce::MAGNITUDE)];

        constexpr float MAX_WHEEL_SPEED = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
        constexpr float K_ROT = 0.5F;

        if (magnitude < POSITION_BUFFER)
        {
            _targetWheelCmd = this->idleCmd();
            _endNodeReached = true;
        }
        else
        {
            float v = MAX_WHEEL_SPEED;
            float omega = K_ROT * yaw;

            float leftCmd = v - omega;
            float rightCmd = v + omega;

            leftCmd = std::clamp(leftCmd, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
            rightCmd = std::clamp(rightCmd, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);

            printf("Left wheel command: %.2f, Right wheel command: %.2f\n", leftCmd, rightCmd);

            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_LEFT)] = leftCmd;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_LEFT)] = leftCmd;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_RIGHT)] = rightCmd;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_RIGHT)] = rightCmd;
        }

        return _targetWheelCmd;
    }

    std::array<float, TO_UNDERLYING(eWheelCmd::eLAST)> rotate(void)
    {
        _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_LEFT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
        _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_LEFT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
        _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_RIGHT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL * -1.0F;
        _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_RIGHT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL * -1.0F;

        return _targetWheelCmd;
    }

    std::array<float, TO_UNDERLYING(eWheelCmd::eLAST)> idleCmd(void)
    {
        _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_LEFT)] = 0.0F;
        _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_LEFT)] = 0.0F;
        _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_RIGHT)] = 0.0F;
        _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_RIGHT)] = 0.0F;

        return _targetWheelCmd;
    }

    std::array<float, TO_UNDERLYING(eWheelCmd::eLAST)> getToHeading(eRotationDirection rotationDirection_)
    {
        if (rotationDirection_ == eRotationDirection::CLOCKWISE)
        {
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_LEFT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_LEFT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_RIGHT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL * -1.0F;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_RIGHT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL * -1.0F;
        }
        else if (rotationDirection_ == eRotationDirection::COUNTERCLOCKWISE)
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
        double currentLatRad = _currentLat * std::numbers::pi / 180.0;
        double targetLatRad = _targetLat * std::numbers::pi / 180.0;
        double deltaLonRad = (_targetLon - _currentLon) * std::numbers::pi / 180.0;

        double y = sin(deltaLonRad) * cos(targetLatRad);
        double x = cos(currentLatRad) * sin(targetLatRad) - sin(currentLatRad) * cos(targetLatRad) * cos(deltaLonRad);

        double bearing = atan2(y, x);
        bearing = bearing * 180.0 / std::numbers::pi;

        if (bearing < 0.0)
        {
            bearing += 360.0;
        }

        bearing = 90.0 - bearing;
        if (bearing < 0.0)
        {
            bearing += 360.0;
        }

        return static_cast<float>(bearing);
    }

    float getDistanceBetweenPoints(void)
    {
        float lat1Rad = _currentLat * std::numbers::pi / 180.0F;
        float lat2Rad = _targetLat * std::numbers::pi / 180.0F;
        float deltaLatRad = (lat2Rad - lat1Rad);
        float deltaLonRad = (_targetLon - _currentLon) * std::numbers::pi / 180.0F;

        float a = sin(deltaLatRad / 2.0F) * sin(deltaLatRad / 2.0F)
                  + cos(lat1Rad) * cos(lat2Rad) * sin(deltaLonRad / 2.0F) * sin(deltaLonRad / 2.0F);
        float c = 2.0F * atan2(sqrt(a), sqrt(1 - a));

        return EARTH_RADIUS_METERS * c;
    }

    std::array<float, TO_UNDERLYING(eWheelCmd::eLAST)> setWheelCmd(void)
    {
        double bearing = this->computeBearing();

        if (this->getDistanceBetweenPoints() <= POSITION_BUFFER)
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
            if (this->computeRotationDirection(bearing) == eRotationDirection::CLOCKWISE)
            {
                _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_LEFT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
                _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_LEFT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
                _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_RIGHT)]
                    = Constants::DriveTrain::SPEED_FACTOR_NORMAL * RECTIFICATION_FACTOR;
                _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_RIGHT)]
                    = Constants::DriveTrain::SPEED_FACTOR_NORMAL * RECTIFICATION_FACTOR;
            }
            else if (this->computeRotationDirection(bearing) == eRotationDirection::COUNTERCLOCKWISE)
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

  private:
    std::deque<float> _yawHistory;
    static constexpr size_t MOVING_AVERAGE_WINDOW_SIZE = 10;
    static constexpr float EXPONENTIAL_FACTOR = 0.3f;

  public:
    float normalizeAngle(float angle)
    {
        while (angle > std::numbers::pi)
            angle -= 2.0f * std::numbers::pi;
        while (angle < -std::numbers::pi)
            angle += 2.0f * std::numbers::pi;
        return angle;
    }

    float applyMovingAverageToYaw(float newYaw)
    {
        newYaw = normalizeAngle(newYaw);
        _yawHistory.push_back(newYaw);
        if (_yawHistory.size() > MOVING_AVERAGE_WINDOW_SIZE)
        {
            _yawHistory.pop_front();
        }

        float sumSin = 0.0f;
        float sumCos = 0.0f;
        for (float yawVal : _yawHistory)
        {
            sumSin += std::sin(yawVal);
            sumCos += std::cos(yawVal);
        }
        return normalizeAngle(std::atan2(sumSin / _yawHistory.size(), sumCos / _yawHistory.size()));
    }

    std::array<float, TO_UNDERLYING(eForceVector::eLAST)> calculateAttractiveForces(float distanceToGoal_, float bearingDeg_)
    {
        float bearingRad = bearingDeg_ * std::numbers::pi / 180.0f;
        return {distanceToGoal_ * std::cos(bearingRad), distanceToGoal_ * std::sin(bearingRad)};
    }

    std::array<float, TO_UNDERLYING(eForceVector::eLAST)> calculateRepulsiveForces(const std::vector<int8_t>& costmapData_)
    {
        std::array<float, TO_UNDERLYING(eForceVector::eLAST)> repulsiveForces = {0.0f, 0.0f};
        int midX = LIDAR_CONFIG::COSTMAP::MAP_WIDTH / 2;
        int midY = LIDAR_CONFIG::COSTMAP::MAP_HEIGHT / 2;
        int radius = static_cast<int>(LIDAR_CONFIG::NAVIGATION::INFLUENCE_DISTANCE / LIDAR_CONFIG::COSTMAP::MAP_RESOLUTION);
        for (int dy = -radius; dy <= radius; ++dy)
        {
            for (int dx = -radius; dx <= radius; ++dx)
            {
                int x = midX + dx;
                int y = midY + dy;
                if (x < 0 || x >= LIDAR_CONFIG::COSTMAP::MAP_WIDTH || y < 0 || y >= LIDAR_CONFIG::COSTMAP::MAP_HEIGHT)
                    continue;
                int idx = y * LIDAR_CONFIG::COSTMAP::MAP_WIDTH + x;
                int cost = costmapData_[idx];
                if (cost < 100)
                    continue;
                float ox = dx * LIDAR_CONFIG::COSTMAP::MAP_RESOLUTION;
                float oy = dy * LIDAR_CONFIG::COSTMAP::MAP_RESOLUTION;
                float dist = std::hypot(ox, oy);
                if (dist < 1e-6f || dist > LIDAR_CONFIG::NAVIGATION::INFLUENCE_DISTANCE)
                    continue;
                float bearing = std::atan2(oy, ox);
                float normDist = dist / LIDAR_CONFIG::NAVIGATION::INFLUENCE_DISTANCE;
                float mag = LIDAR_CONFIG::NAVIGATION::REPULSIVE_GAIN * (cost / 100.0f) * std::exp(-EXPONENTIAL_FACTOR * normDist)
                            / (dist * dist);
                repulsiveForces[0] -= mag * std::cos(bearing);
                repulsiveForces[1] -= mag * std::sin(bearing);
            }
        }
        return repulsiveForces;
    }

    std::array<float, TO_UNDERLYING(eForceVector::eLAST)> calculateTotalForces(
        const std::array<float, TO_UNDERLYING(eForceVector::eLAST)>& attractive,
        const std::array<float, TO_UNDERLYING(eForceVector::eLAST)>& repulsive)
    {
        return {attractive[0] + repulsive[0], attractive[1] + repulsive[1]};
    }

    std::array<float, TO_UNDERLYING(eTotalForce::eLAST)> computeHeading(const std::vector<int8_t>& costmapData_)
    {
        float bear = computeBearing();
        float dist = getDistanceBetweenPoints();
        auto attr = calculateAttractiveForces(dist, bear);
        auto rep = calculateRepulsiveForces(costmapData_);
        auto tot = calculateTotalForces(attr, rep);
        float mag = std::hypot(tot[0], tot[1]);
        float raw = std::atan2(tot[1], tot[0]);
        float smooth = applyMovingAverageToYaw(raw);
        return {mag, smooth};
    }
};

#endif  // NAVIGATION_CONTROLLER_HPP
