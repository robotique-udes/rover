#include "goal_manager_node.hpp"
#include "rover_lib2/helpers/constants.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <queue>
#include <vector>
#include <limits>
#include <cmath>

/* // TODO
 * Add sanity cehck for gps and heading
 * Add sanity check for aruco detection
 * Add teleop priority
 * Add error handling
 * Add config file for lidar
 */

GoalManager::GoalManager():
    Node("Goal_manager")
{
    _sub_currentGps = this->create_subscription<rover_msgs::msg::Gps>(TOPIC_GPS_NAME,
                                                                      QOS_DEFAULT,
                                                                      [this](const rover_msgs::msg::Gps& gpsMsg_)
                                                                      {
                                                                          this->CB_currentGps(gpsMsg_);
                                                                      });

    _sub_arucoDetection = this->create_subscription<rover_msgs::msg::Aruco>(TOPIC_ARUCO_DETECTED,
                                                                            QOS_DEFAULT,
                                                                            [this](const rover_msgs::msg::Aruco& arucoMsg_)
                                                                            {
                                                                                this->CB_aruco(arucoMsg_);
                                                                            });

    _sub_costmap = this->create_subscription<nav_msgs::msg::OccupancyGrid>(TOPIC_COSTMAP_NAME,
                                                                           QOS_DEFAULT,
                                                                           [this](const nav_msgs::msg::OccupancyGrid& costmapMsg_)
                                                                           {
                                                                               this->CB_costmap(costmapMsg_);
                                                                           });

    _pub_auto_cmd = this->create_publisher<rover_msgs::msg::PropulsionMotor>(TOPIC_WHEEL_CMD_NAME, QOS_DEFAULT);
    _pub_marker = this->create_publisher<visualization_msgs::msg::MarkerArray>(TOPIC_MARKER, QOS_DEFAULT);
    _pub_avoidanceArrow = this->create_publisher<visualization_msgs::msg::Marker>(TOPIC_ARROW, QOS_DEFAULT);

    _srv_desiredGps = this->create_service<rover_msgs::srv::DesiredGpsPosition>(
        SRV_GOAL_NAME,
        [this](const rover_msgs::srv::DesiredGpsPosition::Request::SharedPtr request_,
               rover_msgs::srv::DesiredGpsPosition::Response::SharedPtr response_)
        {
            this->CB_desiredGps(request_, response_);
        });
    _timer = this->create_wall_timer(std::chrono::milliseconds(PUBLISHER_PERIOD_MS),
                                     [this]()
                                     {
                                         this->driveTrainPublisher();
                                     });
    _srv_arucoDetection = this->create_client<rover_msgs::srv::ArucoDetection>(SERVICE_SERVER_NAME);
    _navigationController.headingBuffer_ = HEADING_BUFFER;  // TODO make this cleaner
}

void GoalManager::CB_aruco(const rover_msgs::msg::Aruco& arucoMsg_)
{
    if (arucoMsg_.valid)
    {
        this->_arucoDetected = true;
    }
}

void GoalManager::CB_costmap(const nav_msgs::msg::OccupancyGrid& grid)
{
    const double AVOID_WINDOW_FORWARD_M = 2.0;
    const double AVOID_WINDOW_LATERAL_M = 0.3;
    const double AVOIDANCE_MARGIN_DEG = 15.0;
    constexpr int THRESH = 100;
    constexpr int MIN_CLUSTER_SIZE = 10;
    constexpr double MAX_CLUSTER_DIST_C = 1.5;

    const auto& info = grid.info;
    int width = info.width;
    int height = info.height;
    double res = info.resolution;
    _costmapData = grid.data;

    int gx = int((0.0 - info.origin.position.x) / res);
    int gy = int((0.0 - info.origin.position.y) / res);
    gx = std::clamp(gx, 0, width - 1);
    gy = std::clamp(gy, 0, height - 1);

    int forward_cells = std::max(1, int(AVOID_WINDOW_FORWARD_M / res));
    int lateral_cells = std::max(1, int(AVOID_WINDOW_LATERAL_M / res));

    int x_min = std::clamp(gx + 1, 0, width - 1);
    int x_max = std::clamp(gx + forward_cells, 0, width - 1);
    int y_min = std::clamp(gy - lateral_cells, 0, height - 1);
    int y_max = std::clamp(gy + lateral_cells, 0, height - 1);

    std::vector<std::pair<int, int>> occupied;
    for (int x = x_min; x <= x_max; ++x)
    {
        for (int y = y_min; y <= y_max; ++y)
        {
            int idx = y * width + x;
            if (_costmapData[idx] >= THRESH)
            {
                occupied.emplace_back(x, y);
            }
        }
    }

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
        {
            clusters.push_back(std::move(cluster));
        }
    }

    visualization_msgs::msg::MarkerArray ma;

    visualization_msgs::msg::Marker clear;
    clear.header = grid.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(clear);

    int id = 0;
    for (auto& cl : clusters)
    {
        visualization_msgs::msg::Marker m;
        m.header = grid.header;
        m.ns = "obstacle_clusters";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::CUBE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = res;
        m.scale.y = res;
        m.scale.z = 0.2;
        m.color.r = 1.0;
        m.color.g = 0.5;
        m.color.b = 0.0;
        m.color.a = 0.7;

        for (auto [cx, cy] : cl)
        {
            geometry_msgs::msg::Point p;
            p.x = cx * res + info.origin.position.x + res / 2.0;
            p.y = cy * res + info.origin.position.y + res / 2.0;
            p.z = info.origin.position.z + 0.1;
            m.points.push_back(p);
        }
        ma.markers.push_back(m);
    }

    _pub_marker->publish(ma);

    if (!clusters.empty())
    {
        size_t nearest = 0;
        double best_d = std::numeric_limits<double>::infinity();
        double ccx = 0, ccy = 0;

        for (size_t i = 0; i < clusters.size(); ++i)
        {
            double sx = 0, sy = 0;
            for (auto& cell : clusters[i])
            {
                sx += cell.first;
                sy += cell.second;
            }
            double cx = sx / clusters[i].size();
            double cy = sy / clusters[i].size();
            double dx = (cx - gx) * res;
            double dy = (cy - gy) * res;
            double d = std::hypot(dx, dy);
            if (d < best_d)
            {
                best_d = d;
                nearest = i;
                ccx = cx;
                ccy = cy;
            }
        }

        int min_y = INT_MAX, max_y = INT_MIN;
        for (auto& cell : clusters[nearest])
        {
            min_y = std::min(min_y, cell.second);
            max_y = std::max(max_y, cell.second);
        }
        double width_m = (max_y - min_y + 1) * res;

        double half_rad = std::atan2(width_m * 0.5, best_d);
        double half_deg = half_rad * 180.0 / M_PI;

        double steer = half_deg + AVOIDANCE_MARGIN_DEG;
        double rel_y = (ccy - gy) * res;
        float desired_heading = (rel_y > 0) ? static_cast<float>(-steer) : static_cast<float>(steer);

        _obstacleHeading = -desired_heading;

        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = "base_link";
        arrow.header.stamp = now();
        arrow.ns = "avoidance";
        arrow.id = 0;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.pose.position.x = 0;
        arrow.pose.position.y = 0;
        arrow.pose.position.z = 0;
        {
            tf2::Quaternion q;
            q.setRPY(0, 0, desired_heading * M_PI / 180.0);
            arrow.pose.orientation.x = q.x();
            arrow.pose.orientation.y = q.y();
            arrow.pose.orientation.z = q.z();
            arrow.pose.orientation.w = q.w();
        }
        arrow.scale.x = 1.0;
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.1;
        arrow.color.r = 0.0;
        arrow.color.g = 1.0;
        arrow.color.b = 0.0;
        arrow.color.a = 0.8;
        arrow.lifetime = rclcpp::Duration::from_seconds(0.1);

        _pub_avoidanceArrow->publish(arrow);
    }
}

void GoalManager::CB_currentGps(const rover_msgs::msg::Gps& gpsMsg_)
{
    std::array<float, TO_UNDERLYING(NavigationController::eGpsData::eLAST)> currentGpsData
        = {gpsMsg_.latitude, gpsMsg_.longitude, gpsMsg_.heading};

    _navigationController.getCurrentGpsData(currentGpsData);
}

void GoalManager::CB_desiredGps(const rover_msgs::srv::DesiredGpsPosition::Request::SharedPtr request_,
                                rover_msgs::srv::DesiredGpsPosition::Response::SharedPtr response_)
{
    RCLCPP_INFO(this->get_logger(),
                "Received desired GPS position: lat=%.6f, lon=%.6f",
                request_->desired_latitude,
                request_->desired_longitude);

    std::array<float, TO_UNDERLYING(NavigationController::eGpsData::eLAST)> desiredGpsData = {
        request_->desired_latitude,
        request_->desired_longitude,
        0.0F  // Heading is not used in this context
    };

    this->goalReached = false;
    this->goalRequested = true;

    // TODO : Add check
    response_->success = true;

    _navigationController.getDesiredGpsData(desiredGpsData);
}

void GoalManager::visualizeHeading(float heading_)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "base_link";
    m.header.stamp = now();
    m.ns = "avoidance";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, heading_ * M_PI / 180.0);
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();

    m.scale.x = 1.0;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 0.8;
    m.lifetime = rclcpp::Duration::from_seconds(0.1);

    _pub_avoidanceArrow->publish(m);
}

void GoalManager::driveTrainPublisher(void)
{
    // TODO Write getters/setters instead of accessing variables
    switch (_state)
    {
        case (eState::IDLE):
            RCLCPP_INFO(this->get_logger(), "State: IDLE");
            if (goalRequested)
            {
                // _state = eState::ROTATING;
                _state = eState::NAVIGATING_TO_POINT;
            }
            else
            {
                this->_targetWheelCmd = _navigationController.idleCmd();
            }
            break;
        case (eState::ROTATING):
            RCLCPP_INFO(this->get_logger(), "State: ROTATING");
            if (_navigationController._desiredHeadingReached)
            {
                _state = eState::NAVIGATING_TO_POINT;
            }
            else
            {
                double bearing = _navigationController.computeBearing();
                float headingDiff = std::abs(bearing - _navigationController._currentHeading);

                NavigationController::eRotationDirection rotationDirection
                    = _navigationController.computeRotationDirection(headingDiff);

                this->_targetWheelCmd = _navigationController.getToHeading(rotationDirection);
            }
            break;
        case (eState::NAVIGATING_TO_POINT):
            RCLCPP_INFO(this->get_logger(), "State: NAVIGATING_TO_POINT");
            if (_navigationController._endNodeReached)
            {
                auto arucoRequest = std::make_shared<rover_msgs::srv::ArucoDetection::Request>();
                arucoRequest->command = rover_msgs::srv::ArucoDetection::Request::START;
                arucoRequest->camera_url = Constants::CameraInfo::CAMERA_URL_MAP.at(("Main"));  // TODO Make better
                goalRequested = false;
                goalReached = true;
                _state = eState::DETECTING_ARUCO;
            }
            else
            {
                if (_navigationController.obstacleDetected(_costmapData))
                {
                    _state = eState::AVOID_OBSTACLE;
                }
                else
                {
                    this->_targetWheelCmd = _navigationController.navigateToPoint();
                }
            }
            break;
        case (eState::AVOID_OBSTACLE):
            RCLCPP_INFO(this->get_logger(), "State: AVOID_OBSTACLE");
            if (!_navigationController.obstacleDetected(_costmapData))
            {
                _state = eState::ROTATING;
            }
            else
            {
                if (_obstacleHeading < 0.0)
                {
                    _obstacleHeading += 360.0F;
                }

                NavigationController::eRotationDirection rotationDirection
                    = _navigationController.computeRotationDirection(_obstacleHeading);

                this->_targetWheelCmd = _navigationController.getToHeading(rotationDirection);
            }
            break;
        case (eState::DETECTING_ARUCO):
            if (!this->_arucoDetected)
            {
                _targetWheelCmd = _navigationController.rotate();
            }
            else
            {
                _targetWheelCmd = _navigationController.idleCmd();
                goalRequested = false;
                goalReached = true;

                _state = eState::IDLE;
                // TODO END NODE REACHED LOGIC
            }
    }

    auto msg = rover_msgs::msg::PropulsionMotor();
    msg.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_FRONT_LEFT] = _targetWheelCmd[0];
    msg.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_REAR_LEFT] = _targetWheelCmd[1];
    msg.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_FRONT_RIGHT] = _targetWheelCmd[2];
    msg.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_REAR_RIGHT] = _targetWheelCmd[3];

    _pub_auto_cmd->publish(msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalManager>());
    rclcpp::shutdown();
}