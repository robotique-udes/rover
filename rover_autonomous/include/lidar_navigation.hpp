#ifndef LIDAR_NAVIGATION_HPP
#define LIDAR_NAVIGATION_HPP

#include <memory>
#include <cmath>
#include <array>
#include <algorithm>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include <navigation_controller.hpp>

class LidarNavigation
{
    static constexpr float INFLUENCE_DISTANCE = 1.5F;
    static constexpr float REPULSIVE_GAIN = 0.8F;

  private:
    std::array<float, TO_UNDERLYING(eWheelCmd::eLAST)> computeLidarNav(const sensor_msgs::msg::PointCloud2& msg_)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(msg_, cloud);

        // TODO determine necessary axis
        float forceX = 0.0F;
        float forceY = 0.0F;

        for (const auto& pt : cloud.points)
        {
            float x = pt.x;
            float y = pt.y;

            float r = std::hypot(x, y);

            if (r < INFLUENCE_DISTANCE)
            {
                float magnitude = REPULSIVE_GAIN * (1.0F / r - 1.0F / INFLUENCE_DISTANCE) / (r * r);

                forceX += -magnitude * (x / r);
                forceY += -magnitude * (y / r);
            }
        }

        float normalForce = std::hypot(forceX, forceY);

        if (normalForce < 1e-6F)
        {
            // No repulsive force detected, return normal speed
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_LEFT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_LEFT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_RIGHT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_RIGHT)] = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
        }
        else
        {
            float steer = std::atan2(forceY, forcex);

            float forwardFactor = Constants::DriveTrain::SPEED_FACTOR_NORMAL * std::max(0.0F, 1.0F - normalForce);

            float angularFactor = Constants::DriveTrain::TURN_GAIN * steer;

            float leftFactor = forwardFactor - angularFactor;
            float rightFactor = forwardFactor + angularFactor;

            // TODO check for constrain in helpers
            leftFactor = std::clamp(leftFactor, Constants::DriveTrain::SPEED_FACTOR_CRAWLER, Constants::DriveTrain::SPEED_FACTOR_NORMAL);
            rightFactor = std::clamp(rightFactor, Constants::DriveTrain::SPEED_FACTOR_CRAWLER, Constants::DriveTrain::SPEED_FACTOR_NORMAL);

            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_LEFT)] = leftFactor;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_LEFT)] = leftFactor;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::FRONT_RIGHT)] = rightFactor;
            _targetWheelCmd[TO_UNDERLYING(eWheelCmd::REAR_RIGHT)] = rightFactor;
        }

        return _targetWheelCmd;
    }
};

#endif