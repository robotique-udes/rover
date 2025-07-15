#include "goal_manager_node.hpp"

#include "rover_lib2/helpers/constants.hpp"
#include "rover_lib2/helpers/macros.hpp"

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

// Simple 2D vector
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
    _pub_marker = this->create_publisher<visualization_msgs::msg::Marker>(TOPIC_MARKER, QOS_DEFAULT);

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

void GoalManager::visualizeHeading(std::array<float, TO_UNDERLYING(NavigationController::eTotalForce::eLAST)> totalForces_)
{
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "base_link";
    arrow.header.stamp = this->now();
    arrow.ns = "potential_field";
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;

    arrow.pose.position.x = 0.0;
    arrow.pose.position.y = 0.0;
    arrow.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, totalForces_[TO_UNDERLYING(NavigationController::eTotalForce::YAW)]);
    arrow.pose.orientation.x = q.x();
    arrow.pose.orientation.y = q.y();
    arrow.pose.orientation.z = q.z();
    arrow.pose.orientation.w = q.w();

    float length = std::min(totalForces_[TO_UNDERLYING(NavigationController::eTotalForce::MAGNITUDE)], 1.0F);

    arrow.scale.x = length;
    arrow.scale.y = 0.05F;
    arrow.scale.z = 0.05F;

    arrow.color.r = 1.0F;
    arrow.color.g = 0.0F;
    arrow.color.b = 0.0F;
    arrow.color.a = 1.0F;

    _pub_marker->publish(arrow);
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
    _currentCostmap = grid;
}

void GoalManager::CB_currentGps(const rover_msgs::msg::Gps& gpsMsg_)
{
    _currentGpsData = {gpsMsg_.latitude, gpsMsg_.longitude, gpsMsg_.heading};

    _navigationController.getCurrentGpsData(_currentGpsData);
}

void GoalManager::CB_desiredGps(const rover_msgs::srv::DesiredGpsPosition::Request::SharedPtr request_,
                                rover_msgs::srv::DesiredGpsPosition::Response::SharedPtr response_)
{
    RCLCPP_INFO(this->get_logger(),
                "Received desired GPS position: lat=%.6f, lon=%.6f",
                request_->desired_latitude,
                request_->desired_longitude);

    _desiredGpsData = {request_->desired_latitude, request_->desired_longitude, 0.0F};

    this->goalReached = false;
    this->goalRequested = true;

    response_->success = true;

    _navigationController.getDesiredGpsData(_desiredGpsData);
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
                _state = eState::ROTATING;
            }
            else
            {
                this->_targetWheelCmd = _navigationController.idleCmd();
            }
            break;
        case (eState::ROTATING):
            RCLCPP_INFO(this->get_logger(), "State: ROTATING");
            RCLCPP_INFO(this->get_logger(), "desired heading reached: %s",
                        _navigationController._desiredHeadingReached ? "true" : "false");
            
            if (_navigationController._desiredHeadingReached)
            {
                _state = eState::NAVIGATING_TO_POINT;
            }
            else
            {
                double bearing = _navigationController.computeBearing();
                float headingDiff = std::abs(bearing - _navigationController._currentHeading);

                RCLCPP_INFO(this->get_logger(), "heading diff", headingDiff);

                NavigationController::eRotationDirection rotationDirection
                    = _navigationController.computeRotationDirection(headingDiff);

                this->_targetWheelCmd = _navigationController.getToHeading(rotationDirection);
            }
            break;
        case (eState::NAVIGATING_TO_POINT):
            RCLCPP_INFO(this->get_logger(), "State: NAVIGATING_TO_POINT");
            if (_navigationController._endNodeReached)
            {
                RCLCPP_INFO(this->get_logger(), "Point reached");

                // auto arucoRequest = std::make_shared<rover_msgs::srv::ArucoDetection::Request>();
                // arucoRequest->command = rover_msgs::srv::ArucoDetection::Request::START;
                // arucoRequest->camera_url = Constants::CameraInfo::CAMERA_URL_MAP.at(("Main"));  // TODO Make better
                // goalRequested = false;
                // goalReached = true;
                // _state = eState::DETECTING_ARUCO;
            }
            else
            {
                std::array<float, TO_UNDERLYING(NavigationController::eTotalForce::eLAST)> totalForces
                    = _navigationController.computeHeading(_currentCostmap.data);
                _targetWheelCmd = _navigationController.navigate(totalForces);

                // DEBUG PURPOSES
                this->visualizeHeading(totalForces);
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