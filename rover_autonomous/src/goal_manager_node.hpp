#ifndef GOAL_MANAGER_NODE_HPP
#define GOAL_MANAGER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/gps.hpp"
#include <rover_msgs/msg/propulsion_motor.hpp>
#include "rover_msgs/msg/aruco.hpp"
#include "rover_msgs/srv/desired_gps_position.hpp"
#include "rover_msgs/srv/aruco_detection.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "navigation_controller.hpp"

class GoalManager : public rclcpp::Node
{
  public:
    // Topics & services
    static constexpr const char* TOPIC_GPS_NAME = "/rover/gps/position";
    static constexpr const char* TOPIC_COSTMAP_NAME = "/rover/autonomous/costmap";
    static constexpr const char* TOPIC_WHEEL_CMD_NAME = "/rover/drive_train/wheels_cmd_auto";
    static constexpr const char* TOPIC_MARKER = "/rover/autonomous/marker";
    static constexpr const char* TOPIC_ARROW = "/rover/autonomous/arrow";
    static constexpr const char* TOPIC_ARUCO_DETECTED = "aruco";
    static constexpr const char* SRV_GOAL_NAME = "/rover/goal/position";
    static constexpr const char* SERVICE_SERVER_NAME = "/rover/cameras/aruco_detection_management";

    static constexpr uint64_t PUBLISHER_PERIOD_MS = 200UL;
    static constexpr float HEADING_BUFFER = 1.0F;
    static constexpr size_t MAG_WINDOW_SIZE = 10;

    enum class eState
    {
        IDLE = 0,
        ROTATING,
        NAVIGATING_TO_POINT,
        DETECTING_ARUCO,
        AVOID_OBSTACLE,
        ERROR
    };

    enum class eForceVector
    {
        FORCE_X = 0,
        FORCE_Y = 1,
        eLAST
    };

    GoalManager();
    ~GoalManager() override = default;

    NavigationController _navigationController;

  private:
    rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr _sub_currentGps;
    rclcpp::Subscription<rover_msgs::msg::Aruco>::SharedPtr _sub_arucoDetection;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _sub_costmap;

    rclcpp::Publisher<rover_msgs::msg::PropulsionMotor>::SharedPtr _pub_auto_cmd;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _pub_marker;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _pub_potentialField;

    rclcpp::Service<rover_msgs::srv::DesiredGpsPosition>::SharedPtr _srv_desiredGps;
    rclcpp::Client<rover_msgs::srv::ArucoDetection>::SharedPtr _srv_arucoDetection;
    rclcpp::TimerBase::SharedPtr _timer;

    eState _state{eState::IDLE};
    bool goalRequested{false};
    bool goalReached{false};
    bool _arucoDetected{false};

    nav_msgs::msg::OccupancyGrid _currentCostmap;

    std::array<float, TO_UNDERLYING(NavigationController::eWheelCmd::eLAST)> _targetWheelCmd{{0, 0, 0, 0}};
    float _obstacleHeading{0.0F};

    std::array<double, TO_UNDERLYING(NavigationController::eGpsData::eLAST)> _currentGpsData;
    std::array<double, TO_UNDERLYING(NavigationController::eGpsData::eLAST)> _desiredGpsData;

    void CB_currentGps(const rover_msgs::msg::Gps& gpsMsg);
    void CB_aruco(const rover_msgs::msg::Aruco& arucoMsg);
    void CB_costmap(const nav_msgs::msg::OccupancyGrid& costmapMsg);
    void CB_desiredGps(const rover_msgs::srv::DesiredGpsPosition::Request::SharedPtr request,
                       rover_msgs::srv::DesiredGpsPosition::Response::SharedPtr response);

    void driveTrainPublisher();
    void visualizeHeading(std::array<float, TO_UNDERLYING(NavigationController::eTotalForce::eLAST)> totalForces_);
};

#endif  // GOAL_MANAGER_NODE_HPP
