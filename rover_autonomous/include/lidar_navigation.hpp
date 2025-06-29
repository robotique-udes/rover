#ifndef LIDAR_NAVIGATION_HPP
#define LIDAR_NAVIGATION_HPP

#include <memory>
#include <cmath>
#include <array>
#include <algorithm>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include <navigation_controller.hpp>

class LidarNavigation
{
  public:
    enum class eWheelCmd
    {
        FRONT_LEFT = 0,
        REAR_LEFT = 1,
        FRONT_RIGHT = 2,
        REAR_RIGHT = 3,
        eLAST
    };

    // Repulsion parameters
    static constexpr float INFLUENCE_DISTANCE = 1.5F;
    static constexpr float MAX_DETECTION_DISTANCE = 2.0F;
    static constexpr float REPULSIVE_GAIN = 0.8F;
    static constexpr float TURN_GAIN = 1.0F;

    // Costmap parameters
    static constexpr float GRID_SIZE_M = 10.0F;      // 10m x 10m
    static constexpr float GRID_RES_METERS = 0.05F;  // 5cm resolution
    static constexpr int GRID_CELLS = int(GRID_SIZE_M / GRID_RES_METERS);

    LidarNavigation()
    {
        // Prepare static template for occupancy grid
        _grid_template.header.frame_id = "unilidar_lidar";
        _grid_template.info.resolution = GRID_RES_METERS;
        _grid_template.info.width = GRID_CELLS;
        _grid_template.info.height = GRID_CELLS;
        _grid_template.info.origin.position.x = -GRID_SIZE_M / 2.0;
        _grid_template.info.origin.position.y = -GRID_SIZE_M / 2.0;
        _grid_template.info.origin.orientation.w = 1.0;
        _grid_template.data.resize(GRID_CELLS * GRID_CELLS);
    }

    /**
     * Compute wheel speed commands based purely on repulsive forces.
     * @param msg Input 3D point cloud
     * @return Array of four wheel speed factors
     */
    std::array<float, TO_UNDERLYING(eWheelCmd::eLAST)> computeWheelCommands(const sensor_msgs::msg::PointCloud2& msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(msg, cloud);

        float fx = 0.0F, fy = 0.0F;
        for (auto const& pt : cloud.points)
        {
            float x = pt.y, y = pt.x;
            float r = std::hypot(x, y);
            if (r > 0.0F && r < INFLUENCE_DISTANCE)
            {
                float mag = REPULSIVE_GAIN * (1.0F / r - 1.0F / INFLUENCE_DISTANCE) / (r * r);
                fx += -mag * (x / r);
                fy += -mag * (y / r);
            }
        }
        float norm = std::hypot(fx, fy);

        std::array<float, TO_UNDERLYING(eWheelCmd::eLAST)> cmds;
        if (norm < 1e-6F)
        {
            cmds.fill(Constants::DriveTrain::SPEED_FACTOR_NORMAL);
        }
        else
        {
            float steer = std::atan2(fy, fx);
            float forward = Constants::DriveTrain::SPEED_FACTOR_NORMAL * std::max(0.0F, 1.0F - norm);
            float angular = TURN_GAIN * steer;
            float left = forward - angular;
            float right = forward + angular;
            left = std::clamp(left, Constants::DriveTrain::SPEED_FACTOR_CRAWLER, Constants::DriveTrain::SPEED_FACTOR_NORMAL);
            right = std::clamp(right, Constants::DriveTrain::SPEED_FACTOR_CRAWLER, Constants::DriveTrain::SPEED_FACTOR_NORMAL);
            cmds[TO_UNDERLYING(eWheelCmd::FRONT_LEFT)] = left;
            cmds[TO_UNDERLYING(eWheelCmd::REAR_LEFT)] = left;
            cmds[TO_UNDERLYING(eWheelCmd::FRONT_RIGHT)] = right;
            cmds[TO_UNDERLYING(eWheelCmd::REAR_RIGHT)] = right;
        }
        return cmds;
    }

    /**
     * Generate a local occupancy grid centered on the robot.
     * @param msg Input 3D point cloud
     * @return OccupancyGrid in "base_link" frame
     */
    nav_msgs::msg::OccupancyGrid buildCostmap(const sensor_msgs::msg::PointCloud2& msg)
    {
        auto grid = _grid_template;  // copy static template
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(msg, cloud);

        std::fill(grid.data.begin(), grid.data.end(), 0);
        for (auto const& pt : cloud.points)
        {
            float rx = pt.y;
            float ry = pt.x;
            float r = std::hypot(rx, ry);
            if (!std::isfinite(r) || r > MAX_DETECTION_DISTANCE)
                continue;
            if (std::fabs(rx) > GRID_SIZE_M / 2 || std::fabs(ry) > GRID_SIZE_M / 2)
                continue;
            int gx = int((rx + GRID_SIZE_M / 2.0F) / GRID_RES_METERS);
            int gy = int((ry + GRID_SIZE_M / 2.0F) / GRID_RES_METERS);
            int idx = gy * GRID_CELLS + gx;
            grid.data[idx] = 100;
        }
        grid.header.stamp = rclcpp::Clock().now();
        return grid;
    }

  private:
    nav_msgs::msg::OccupancyGrid _grid_template;
};

#endif  // LIDAR_NAVIGATION_HPP
