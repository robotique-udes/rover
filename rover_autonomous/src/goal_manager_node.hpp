#ifndef GOAL_MANAGER_NODE_HPP
#define GOAL_MANAGER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/gps.hpp"
#include <rover_msgs/msg/propulsion_motor.hpp>
#include "rover_msgs/msg/aruco.hpp"
#include "rover_msgs/srv/desired_gps_position.hpp"
#include "rover_msgs/srv/aruco_detection.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "navigation_controller.hpp"
#include "lidar_navigation.hpp"

class GoalManager : public rclcpp::Node
{
    static constexpr const char* TOPIC_GPS_NAME = "/rover/gps/position";
    static constexpr const char* TOPIC_LIDAR_POINT_CLOUD = "/unilidar/cloud";
    static constexpr const char* TOPIC_MARKER = "/rover/goal/marker";
    static constexpr const char* SRV_GOAL_NAME = "/rover/goal/position";
    static constexpr const char* TOPIC_WHEEL_CMD_NAME = "/rover/drive_train/wheels_cmd_auto";
    static constexpr const char* SERVICE_SERVER_NAME = "/rover/cameras/aruco_detection_management";
    static constexpr const char* TOPIC_ARUCO_DETECTED = "/rover/cameras/aruco_detected";
    static constexpr const char* TOPIC_OCCUPANCY_GRID = "/rover/goal/occupancy_grid";

    static constexpr uint64_t PUBLISHER_PERIOD_MS = 200UL;
    static constexpr float HEADING_BUFFER = 1.0F;

    // TODO Possibly move this and all occupency map to own class
    static constexpr float OCCUPENCY_GRID_SIZE = 10.0F;
    static constexpr float OCCUPENCY_GRID_RESOLUTION = 0.1F;
    static constexpr int GRID_CELLS = static_cast<int>(OCCUPENCY_GRID_SIZE / OCCUPENCY_GRID_RESOLUTION);

  public:
    enum class eState
    {
        IDLE = 0,
        ROTATING = 1,
        NAVIGATING_TO_POINT = 2,
        DETECTING_ARUCO = 3,
        ERROR
    };
    GoalManager();
    ~GoalManager() = default;

    NavigationController _navigationController;
    LidarNavigation _lidarNavigation;

  private:
    rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr _sub_currentGps;
    rclcpp::Subscription<rover_msgs::msg::Aruco>::SharedPtr _sub_arucoDetection;  
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_pointCloud;
    rclcpp::Publisher<rover_msgs::msg::PropulsionMotor>::SharedPtr _pub_auto_cmd;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _pub_marker;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _pub_map;
    rclcpp::Service<rover_msgs::srv::DesiredGpsPosition>::SharedPtr _srv_desiredGps;
    rclcpp::Client<rover_msgs::srv::ArucoDetection>::SharedPtr _srv_arucoDetection;
    rclcpp::TimerBase::SharedPtr _timer;

    sensor_msgs::msg::PointCloud2 _pointCloudMsg; 

    eState _state = eState::NAVIGATING_TO_POINT;

    bool goalRequested = false;
    bool goalReached = false;
    bool _desiredHeadingReached = false;
    bool _arucoDetected = false;

    std::array<float, TO_UNDERLYING(NavigationController::eWheelCmd::eLAST)> _targetWheelCmd = {0.0F, 0.0F, 0.0F, 0.0F};

  public:
    void CB_currentGps(const rover_msgs::msg::Gps& gpsMsg_);
    void CB_aruco(const rover_msgs::msg::Aruco& arucoMsg_);
    void CB_pointCloud(const sensor_msgs::msg::PointCloud2& pointCloudMsg_);
    void CB_desiredGps(const rover_msgs::srv::DesiredGpsPosition::Request::SharedPtr request_,
                       rover_msgs::srv::DesiredGpsPosition::Response::SharedPtr response_);
    void driveTrainPublisher(void);
    void computeVector(void);
    void updateOccupencyGrid(void);
};

#endif