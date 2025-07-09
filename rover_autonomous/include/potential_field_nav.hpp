#ifndef POTENTIAL_FIELD_NAV_HPP
#define POTENTIAL_FIELD_NAV_HPP

#include "rover_lib2/helpers/constants.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "lidar_config.hpp"
#include "navigation_controller.hpp"

#include <cmath>
#include <deque>

class NavigationController;

class PotentialFieldNav
{
    std::deque<float> _yawHistory;
    static constexpr size_t MOVING_AVERAGE_WINDOW_SIZE = 5;

  public:
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
  
    NavigationController _navigationController;
    
    static constexpr float EXPONENTIAL_FACTOR = 0.5f;

    std::array<float, TO_UNDERLYING(eForceVector::eLAST)> calculateTotalForces(
        std::array<float, TO_UNDERLYING(eForceVector::eLAST)> attractiveForces_,
        std::array<float, TO_UNDERLYING(eForceVector::eLAST)> repulsiveForces_)
    {
        std::array<float, TO_UNDERLYING(eForceVector::eLAST)> totalForces;

        totalForces[TO_UNDERLYING(eForceVector::FORCE_X)]
            = attractiveForces_[TO_UNDERLYING(eForceVector::FORCE_X)] + repulsiveForces_[TO_UNDERLYING(eForceVector::FORCE_X)];
        totalForces[TO_UNDERLYING(eForceVector::FORCE_Y)]
            = attractiveForces_[TO_UNDERLYING(eForceVector::FORCE_Y)] + repulsiveForces_[TO_UNDERLYING(eForceVector::FORCE_Y)];

        return totalForces;
    }

    std::array<float, TO_UNDERLYING(eForceVector::eLAST)> calculateAttractiveForces(float distanceToGoal_, float bearingRad_)
    {
        std::array<float, TO_UNDERLYING(eForceVector::eLAST)> attractiveForces;

        float brearingRad = bearingRad_ * std::numbers::pi / 180.0F;
        attractiveForces[TO_UNDERLYING(eForceVector::FORCE_X)] = distanceToGoal_ * std::cos(brearingRad);
        attractiveForces[TO_UNDERLYING(eForceVector::FORCE_Y)] = distanceToGoal_ * std::sin(brearingRad);

        return attractiveForces;
    }

    std::array<float, TO_UNDERLYING(eForceVector::eLAST)> calculateRepulsiveForces(
        std::vector<int8_t, std::allocator<int8_t>> costmapData_)
    {
        std::array<float, TO_UNDERLYING(eForceVector::eLAST)> repulsiveForces;

        int roverMapX = LIDAR_CONFIG::COSTMAP::MAP_WIDTH / 2;
        int roverMapY = LIDAR_CONFIG::COSTMAP::MAP_HEIGHT / 2;

        int influenceRadiusCells
            = static_cast<int>(LIDAR_CONFIG::NAVIGATION::INFLUENCE_DISTANCE / LIDAR_CONFIG::COSTMAP::MAP_RESOLUTION);

        for (int dy = -influenceRadiusCells; dy <= influenceRadiusCells; ++dy)
        {
            for (int dx = -influenceRadiusCells; dx <= influenceRadiusCells; ++dx)
            {
                int checkX = roverMapX + dx;
                int checkY = roverMapY + dy;

                if (checkX < 0 || checkX >= LIDAR_CONFIG::COSTMAP::MAP_WIDTH || checkY < 0
                    || checkY >= LIDAR_CONFIG::COSTMAP::MAP_HEIGHT)
                {
                    continue;
                }

                int mapIndex = checkY * LIDAR_CONFIG::COSTMAP::MAP_WIDTH + checkX;
                int8_t cost = costmapData_[mapIndex];

                if (cost >= 100.0F)
                {
                    float obstacleX = (checkX - roverMapX) * LIDAR_CONFIG::COSTMAP::MAP_RESOLUTION;
                    float obstacleY = (checkY - roverMapY) * LIDAR_CONFIG::COSTMAP::MAP_RESOLUTION;
                    float distanceToObstacle = std::sqrt(obstacleX * obstacleX + obstacleY * obstacleY);

                    if (distanceToObstacle < 1e-6f || distanceToObstacle > LIDAR_CONFIG::NAVIGATION::INFLUENCE_DISTANCE)
                    {
                        continue;
                    }

                    float bearingToObstacle = std::atan2(obstacleY, obstacleX);

                    float normalizedDistance = distanceToObstacle / LIDAR_CONFIG::NAVIGATION::INFLUENCE_DISTANCE;
                    float forceMagnitude = LIDAR_CONFIG::NAVIGATION::REPULSIVE_GAIN * (cost / 100.0f)
                                           * std::exp(-EXPONENTIAL_FACTOR * normalizedDistance)
                                           / (distanceToObstacle * distanceToObstacle);

                    repulsiveForces[TO_UNDERLYING(eForceVector::FORCE_X)] -= forceMagnitude * std::cos(bearingToObstacle);
                    repulsiveForces[TO_UNDERLYING(eForceVector::FORCE_Y)] -= forceMagnitude * std::sin(bearingToObstacle);
                }
            }
        }

        return repulsiveForces;
    }

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

        if (_yawHistory.size() == 1)
        {
            return newYaw;
        }

        float sumSin = 0.0f;
        float sumCos = 0.0f;

        for (float yaw : _yawHistory)
        {
            sumSin += std::sin(yaw);
            sumCos += std::cos(yaw);
        }

        float avgYaw = std::atan2(sumSin / _yawHistory.size(), sumCos / _yawHistory.size());
        return normalizeAngle(avgYaw);
    }

    std::array<float, TO_UNDERLYING(eTotalForce::eLAST)> computeHeading(std::vector<int8_t, std::allocator<int8_t>> costmapData_)
    {
        std::array<float, TO_UNDERLYING(eTotalForce::eLAST)> result;

        float bearingDeg = _navigationController.computeBearing();
        float distanceToGoal = _navigationController.getDistanceBetweenPoints();

        std::array<float, TO_UNDERLYING(eForceVector::eLAST)> attractiveForce
            = this->calculateAttractiveForces(distanceToGoal, bearingDeg);
        std::array<float, TO_UNDERLYING(eForceVector::eLAST)> repulsiveForces = this->calculateRepulsiveForces(costmapData_);

        std::array<float, TO_UNDERLYING(eForceVector::eLAST)> totalForces
            = this->calculateTotalForces(attractiveForce, repulsiveForces);

        float magnitude
            = std::hypot(totalForces[TO_UNDERLYING(eForceVector::FORCE_X)], totalForces[TO_UNDERLYING(eForceVector::FORCE_Y)]);
        float rawYaw
            = std::atan2(totalForces[TO_UNDERLYING(eForceVector::FORCE_Y)], totalForces[TO_UNDERLYING(eForceVector::FORCE_X)]);

        // Apply moving average to yaw angle
        float smoothedYaw = applyMovingAverageToYaw(rawYaw);

        result[TO_UNDERLYING(eTotalForce::MAGNITUDE)] = magnitude;
        result[TO_UNDERLYING(eTotalForce::YAW)] = smoothedYaw;

        return result;
    }
};

#endif