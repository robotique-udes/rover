#ifndef NAVIGATION_CONTROLLER_HPP
#define NAVIGATION_CONTROLLER_HPP

#include <array>
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

  private:
    std::array<float, TO_UNDERLYING(eGpsData::eLAST)> _currentGpsData = {0.0F, 0.0F, 0.0F};

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
        double bearing = computeBearing();
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

    double computeBearing(void)
    {
        double currentLatRad = _currentLat * std::numbers::pi / 180.0;  // Remove F
        double targetLatRad = _targetLat * std::numbers::pi / 180.0;
        double deltaLonRad = (_targetLon - _currentLon) * std::numbers::pi / 180.0;

        double y = sin(deltaLonRad) * cos(targetLatRad);
        double x = cos(currentLatRad) * sin(targetLatRad) - sin(currentLatRad) * cos(targetLatRad) * cos(deltaLonRad);

        double bearing = atan2(y, x);
        bearing = bearing * 180.0 / std::numbers::pi;  // Remove F

        if (bearing < 0.0)
        {
            bearing += 360.0;  // Remove F
        }

        // Convert from mathematical bearing to navigation bearing
        bearing = 90.0 - bearing;  // Remove F
        if (bearing < 0.0)
        {
            bearing += 360.0;  // Remove F
        }

        return bearing;
    }

    float getDistance(void)
    {
        float lat1Rad = _currentLat * std::numbers::pi / 180.0F;
        float lat2Rad = _targetLat * std::numbers::pi / 180.0F;
        float deltaLatRad = (lat2Rad - lat1Rad);
        float deltaLonRad = (_targetLon - _currentLon) * std::numbers::pi / 180.0F;

        float a = sin(deltaLatRad / 2.0F) * sin(deltaLatRad / 2.0F)
                  + cos(lat1Rad) * cos(lat2Rad) * sin(deltaLonRad / 2.0F) * sin(deltaLonRad / 2.0F);
        float c = 2.0F * atan2(sqrt(a), sqrt(1 - a));

        float distance = EARTH_RADIUS_METERS * c;

        printf("Distance to target: %.2f meters\n", distance);

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

    std::array<float, 2> computeNetForce(const std::vector<int8_t>& costmapData)
    {
        // config unpacking
        const float res = LIDAR_CONFIG::COSTMAP::MAP_RESOLUTION;
        const int width = LIDAR_CONFIG::COSTMAP::MAP_WIDTH;
        const int height = LIDAR_CONFIG::COSTMAP::MAP_HEIGHT;
        const float R0 = LIDAR_CONFIG::NAVIGATION::INFLUENCE_DISTANCE;
        const float Krep = LIDAR_CONFIG::NAVIGATION::REPULSIVE_GAIN;
        const int OCC = 100;  // occupied cell value

        // cluster threshold (min #cells)
        constexpr int MIN_CLUSTER_SIZE = 4;

        // precompute for centering
        const float halfW = width * res * 0.5F;
        const float halfH = height * res * 0.5F;

        // 1) mark all occupied cells
        std::vector<bool> occ(width * height, false);
        for (int idx = 0; idx < width * height; ++idx)
        {
            occ[idx] = (costmapData[idx] == OCC);
        }

        // 2) find clusters via flood-fill
        std::vector<bool> seen(width * height, false), valid(width * height, false);
        std::vector<int> stack;
        stack.reserve(width * height);

        // 4-connected offsets
        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};

        for (int idx0 = 0; idx0 < width * height; ++idx0)
        {
            if (!occ[idx0] || seen[idx0])
                continue;
            // new cluster
            stack.clear();
            stack.push_back(idx0);
            seen[idx0] = true;

            // grow it
            for (size_t k = 0; k < stack.size(); ++k)
            {
                int idx = stack[k];
                int x = idx % width;
                int y = idx / width;
                for (int d = 0; d < 4; ++d)
                {
                    int nx = x + dx[d], ny = y + dy[d];
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height)
                    {
                        int nidx = ny * width + nx;
                        if (occ[nidx] && !seen[nidx])
                        {
                            seen[nidx] = true;
                            stack.push_back(nidx);
                        }
                    }
                }
            }

            // 3) if big enough, mark all members valid
            if ((int)stack.size() >= MIN_CLUSTER_SIZE)
            {
                for (int idx : stack)
                    valid[idx] = true;
            }
        }

        // 4) accumulate forces from valid clusters only
        float fx = 0.0F, fy = 0.0F;
        for (int i = 0; i < height; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
                int idx = i * width + j;
                if (!valid[idx])
                    continue;

                // cell center in robot frame
                float cx = (j + 0.5F) * res - halfW;
                float cy = (i + 0.5F) * res - halfH;
                float r = std::hypot(cx, cy);
                if (r > 0.0F && r < R0)
                {
                    // linear repulsion
                    float mag = Krep * (R0 - r);
                    // push _away_ from obstacle
                    fx += mag * (-cx / r);
                    fy += mag * (-cy / r);
                }
            }
        }

        return {fx, fy};
    }
};

#endif