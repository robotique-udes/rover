#include "goal_manager_node.hpp"
#include "rover_lib2/helpers/constants.hpp"

#include <tf2/LinearMath/Quaternion.h>

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

void GoalManager::CB_aruco(const rover_msgs::msg::Aruco& arucoMsg_)
{
    if (arucoMsg_.valid)
    {
        this->_arucoDetected = true;
    }
}

void GoalManager::CB_costmap(const nav_msgs::msg::OccupancyGrid& occupencyGridMsg_)
{
    _costmapData = occupencyGridMsg_.data;
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
    visualization_msgs::msg::Marker marker;

    auto [fx, fy] = _navigationController.computeNetForce(_costmapData);

    marker.header.frame_id = "base_link";  // in robot frame
    marker.header.stamp = now();
    marker.ns = "heading";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // thickness / head size
    marker.scale.x = 0.02;  // shaft diameter
    marker.scale.y = 0.04;  // head diameter
    marker.scale.z = 0.04;  // head length

    // color
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // start at (0,0), end at (fx,fy)
    geometry_msgs::msg::Point p0, p1;
    p0.x = p0.y = p0.z = 0.0;
    p1.x = fx;
    p1.y = fy;
    p1.z = 0.0;
    marker.points = {p0, p1};

    _pub_marker->publish(marker);
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
                this->computeDeisreHeading();
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