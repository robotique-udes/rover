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
#include "potential_field_nav.hpp"

// Simple 2D vector for flat-map coordinates
struct Vector2D
{
    double x{}, y{};
    Vector2D() = default;
    Vector2D(double x_, double y_):
        x(x_),
        y(y_)
    {
    }
    Vector2D operator+(Vector2D o) const
    {
        return {x + o.x, y + o.y};
    }
    Vector2D operator-(Vector2D o) const
    {
        return {x - o.x, y - o.y};
    }
    Vector2D operator*(double s) const
    {
        return {x * s, y * s};
    }
    double magnitude() const
    {
        return std::hypot(x, y);
    }
    Vector2D normalized() const
    {
        double m = magnitude();
        return (m > 1e-6) ? Vector2D(x / m, y / m) : Vector2D(0, 0);
    }
};

// Parameters for potential-field navigation
struct PotentialFieldParams
{
    double attractive_gain{1.0};
    double repulsive_gain{2.0};
    double repulsive_range{2.0};
    double max_speed{0.5};
    double goal_tolerance{0.1};
    double obstacle_threshold{50.0};
    double force_saturation{10.0};
};

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
    PotentialFieldNav _potentialFieldNav;

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

    PotentialFieldParams _potentialFieldParams;
    Vector2D _currentPosition;
    Vector2D _goalPosition;
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

    Vector2D calculateRepulsiveForce(const Vector2D& current, const nav_msgs::msg::OccupancyGrid& costmap);
    void potentialFieldNavigation();
    void visualizePotentialField(const Vector2D& attractive, const Vector2D& repulsive, const Vector2D& total);
    void visualizeHeading(float heading);
};

#endif  // GOAL_MANAGER_NODE_HPP
