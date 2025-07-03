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
    _costmapData = grid.data;

    // This is for debug purposes
    const auto& info = grid.info;
    int width = info.width;               // columns (world X)
    int height = info.height;             // rows    (world Y)
    double resolution = info.resolution;  // meters/cell

    int gx = int((0.0 - info.origin.position.x) / resolution);
    int gy = int((0.0 - info.origin.position.y) / resolution);
    gx = std::clamp(gx, 0, width - 1);
    gy = std::clamp(gy, 0, height - 1);

    constexpr int DETECT_W = 500;  // ± cells left/right
    constexpr int DETECT_D = 500;  // cells forward
    constexpr int THRESH = 100;    // >=100 = occupied

    std::vector<std::pair<int, int>> occupied;
    int x_min = std::max(0, gx + 1);
    int x_max = std::min(width - 1, gx + DETECT_D);
    int y_min = std::max(0, gy - DETECT_W / 2);
    int y_max = std::min(height - 1, gy + DETECT_W / 2);

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

    constexpr int MIN_CLUSTER_SIZE = 10;        // drop small noise
    constexpr double MAX_CLUSTER_DIST_C = 1.5;  // cells
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

    // clear old markers
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
        // each cube = one costmap cell
        m.scale.x = resolution;
        m.scale.y = resolution;
        m.scale.z = 0.2;
        // pick a single color for clusters
        m.color.r = 1.0;
        m.color.g = 0.5;
        m.color.b = 0.0;
        m.color.a = 0.7;

        // convert each cell to world point
        for (auto [cx, cy] : cl)
        {
            geometry_msgs::msg::Point p;
            p.x = cx * resolution + info.origin.position.x + resolution / 2.0;
            p.y = cy * resolution + info.origin.position.y + resolution / 2.0;
            p.z = info.origin.position.z + 0.1;
            m.points.push_back(p);
        }

        ma.markers.push_back(m);
    }

    this->computeDeisreHeading();
    _pub_marker->publish(ma);
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

void GoalManager::computeDeisreHeading(void)
{
    float avoidanceAngle = 45.0F;

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "base_link";  // your lidar frame
    m.header.stamp = now();
    m.ns = "avoidance";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;

    // Place at lidar origin
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;

    // Yaw the arrow by yaw_deg around Z
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, avoidanceAngle * M_PI / 180.0);
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();

    // Styling
    m.scale.x = 1.0;  // length
    m.scale.y = 0.1;  // shaft width
    m.scale.z = 0.1;  // head width
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
            if (goalRequested)
            {
                _state = eState::ROTATING;
            }
            else
            {
                this->_targetWheelCmd = _navigationController.idleCmd();
            }
            break;
        case (eState::ROTATING):
            if (_navigationController._desiredHeadingReached)
            {
                _state = eState::NAVIGATING_TO_POINT;
            }
            else
            {
                this->_targetWheelCmd = _navigationController.getToHeading();
            }
            break;
        case (eState::NAVIGATING_TO_POINT):
            if (!_navigationController._endNodeReached)
            {
                if (_navigationController.obstacleDetected(_costmapData))
                {
                    RCLCPP_WARN(this->get_logger(), "OBSTACLE");
                }
            }
            else
            {
                auto arucoRequest = std::make_shared<rover_msgs::srv::ArucoDetection::Request>();
                arucoRequest->command = rover_msgs::srv::ArucoDetection::Request::START;
                arucoRequest->camera_url = Constants::CameraInfo::CAMERA_URL_MAP.at(("Main"));  // TODO Make better
                goalRequested = false;
                goalReached = true;
                _state = eState::DETECTING_ARUCO;
            }
            break;
        case (eState::AVOID_OBSTACLE):
            // TODO

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