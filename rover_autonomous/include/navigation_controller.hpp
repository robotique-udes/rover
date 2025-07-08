#ifndef NAVIGATION_CONTROLLER_HPP
#define NAVIGATION_CONTROLLER_HPP

#include <array>
#include <queue>
#include <cmath>
#include <algorithm>

#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include "lidar_config.hpp"

class NavigationController
{
    static constexpr float HEADING_BUFFER = 1.0F;
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

    float headingBuffer_ = HEADING_BUFFER;
    bool _desiredHeadingReached = false;
    bool _endNodeReached = false;
    double _currentLat = 0.0;
    double _currentLon = 0.0;
    double _currentHeading = 0.0;
    double _targetLat = 0.0;
    double _targetLon = 0.0;

  private:
    std::array<float, TO_UNDERLYING(eGpsData::eLAST)> _currentGpsData = {0.0F, 0.0F, 0.0F};

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
        if (headingDiff_ <= this->headingBuffer_)
        {
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

    std::array<float, TO_UNDERLYING(eWheelCmd::eLAST)> rotate(void)
    {
        // TODO Maybe define a SPEED FACTOR AUTO between crawler and normal
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

    std::array<float, TO_UNDERLYING(eWheelCmd::eLAST)> navigateToPoint(void)
    {
        if (_endNodeReached)
        {
            _targetWheelCmd = this->idleCmd();
            return _targetWheelCmd;
        }
        else
        {
            _targetWheelCmd = this->setWheelCmd();
        }

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

        return bearing;
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

        float distance = EARTH_RADIUS_METERS * c;

        return distance;
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

    bool obstacleDetected(const std::vector<int8_t>& costmapData)
    {
        static constexpr int WIDTH = LIDAR_CONFIG::COSTMAP::MAP_WIDTH;
        static constexpr int HEIGHT = LIDAR_CONFIG::COSTMAP::MAP_HEIGHT;
        static constexpr double RES = LIDAR_CONFIG::COSTMAP::MAP_RESOLUTION;
        static constexpr double RANGE = LIDAR_CONFIG::COSTMAP::MAX_RANGE;
        static constexpr double OCC_THRESH = LIDAR_CONFIG::COSTMAP::OCCUPIED_THRESHOLD;
        static constexpr float SIDE_DETECTION_WIDTH = 0.2F;

        int depth_cells = static_cast<int>(RANGE / RES);

        int half_width_cells = static_cast<int>(SIDE_DETECTION_WIDTH / RES);

        int gx = WIDTH / 2;
        int gy = HEIGHT / 2;

        int x_min = std::clamp(gx + 1, 0, WIDTH - 1);
        int x_max = std::clamp(gx + depth_cells, 0, WIDTH - 1);
        int y_min = std::clamp(gy - half_width_cells, 0, HEIGHT - 1);
        int y_max = std::clamp(gy + half_width_cells, 0, HEIGHT - 1);

        int occupied_threshold = static_cast<int>(OCC_THRESH * 100.0);

        std::vector<std::pair<int, int>> occupied;
        for (int x = x_min; x <= x_max; ++x)
            for (int y = y_min; y <= y_max; ++y)
            {
                int idx = y * WIDTH + x;
                if (costmapData[idx] >= occupied_threshold)
                    occupied.emplace_back(x, y);
            }
        if (occupied.empty())
            return false;

        static constexpr int MIN_CLUSTER_SIZE = 10;
        static constexpr double MAX_CLUSTER_DIST_C = 1.5;  // in cells

        std::vector<bool> visited(occupied.size(), false);
        std::vector<std::vector<std::pair<int, int>>> clusters;

        for (size_t i = 0; i < occupied.size(); ++i)
        {
            if (visited[i])
                continue;
            std::vector<std::pair<int, int>> cluster;
            std::queue<size_t> q;
            q.push(i);
            visited[i] = true;

            while (!q.empty())
            {
                size_t idx = q.front();
                q.pop();
                cluster.push_back(occupied[idx]);

                for (size_t j = 0; j < occupied.size(); ++j)
                {
                    if (visited[j])
                        continue;
                    double dx = occupied[idx].first - occupied[j].first;
                    double dy = occupied[idx].second - occupied[j].second;
                    if (std::hypot(dx, dy) <= MAX_CLUSTER_DIST_C)
                    {
                        visited[j] = true;
                        q.push(j);
                    }
                }
            }

            if (cluster.size() >= MIN_CLUSTER_SIZE)
                clusters.emplace_back(std::move(cluster));
        }

        return !clusters.empty();
    }
};

#endif