#include "goal_manager_node.hpp"
#include "rover_lib2/helpers/constants.hpp"

/* // TODO
 * Add sanity cehck for gps and heading
 * Add sanity check for aruco detection
 * Add teleop priority
 * Add error handling
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
    _sub_pointCloud
        = this->create_subscription<sensor_msgs::msg::PointCloud2>(TOPIC_LIDAR_POINT_CLOUD,
                                                                   QOS_DEFAULT,
                                                                   [this](const sensor_msgs::msg::PointCloud2& pointCloudMsg_)
                                                                   {
                                                                       this->CB_pointCloud(pointCloudMsg_);
                                                                   });

    _sub_arucoDetection = this->create_subscription<rover_msgs::msg::Aruco>(TOPIC_ARUCO_DETECTED,
                                                                            QOS_DEFAULT,
                                                                            [this](const rover_msgs::msg::Aruco& arucoMsg_)
                                                                            {
                                                                                this->CB_aruco(arucoMsg_);
                                                                            });

    _pub_auto_cmd = this->create_publisher<rover_msgs::msg::PropulsionMotor>(TOPIC_WHEEL_CMD_NAME, QOS_DEFAULT);
    _pub_marker = this->create_publisher<visualization_msgs::msg::Marker>(TOPIC_MARKER, QOS_DEFAULT);
    _pub_map = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/rover/goal/occupancy_grid", QOS_DEFAULT);

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

    nav_msgs::msg::OccupancyGrid _gridMsgs;

    _gridMsgs.header.frame_id = "unilidar_lidar";
    _gridMsgs.info.resolution = OCCUPENCY_GRID_RESOLUTION;
    _gridMsgs.info.width = GRID_CELLS;
    _gridMsgs.info.height = GRID_CELLS;
    _gridMsgs.info.origin.position.x = -OCCUPENCY_GRID_SIZE / 2.0;
    _gridMsgs.info.origin.position.y = -OCCUPENCY_GRID_SIZE / 2.0;
    _gridMsgs.info.origin.position.z = 0.0;
    _gridMsgs.info.origin.orientation.w = 1.0;
}

void GoalManager::CB_aruco(const rover_msgs::msg::Aruco& arucoMsg_)
{
    if (arucoMsg_.valid)
    {
        this->_arucoDetected = true;
    }
}

void GoalManager::CB_pointCloud(const sensor_msgs::msg::PointCloud2& pointCloudMsg_)
{
    _pointCloudMsg = pointCloudMsg_;

    auto occupancyGrid = _lidarNavigation.buildCostmap(_pointCloudMsg);
    _pub_map->publish(occupancyGrid);
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

void GoalManager::computeVector(void)
{
    float left = _targetWheelCmd[TO_UNDERLYING(NavigationController::eWheelCmd::FRONT_LEFT)];
    float right = _targetWheelCmd[TO_UNDERLYING(NavigationController::eWheelCmd::FRONT_RIGHT)];
    float forwardFactor = 0.5F * (left + right);

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "unilidar_lidar";
    m.header.stamp = now();
    m.ns = "commanded_direction";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.scale.x = 0.05;
    m.scale.y = 0.10;
    m.scale.z = 0.10;

    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.color.a = 1.0;

    geometry_msgs::msg::Point p0, p1;
    p0.x = p0.y = p0.z = 0.0;

    p1.x = forwardFactor;
    p1.y = 0.0;
    p1.z = 0.0;

    m.points = {p0, p1};

    _pub_marker->publish(m);
}

void GoalManager::driveTrainPublisher(void)
{
    // TODO Write getters/setters instead of accessing variables
    switch (_state)
    {
        case (eState::IDLE):
            RCLCPP_INFO(this->get_logger(), "IDLE");
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
            RCLCPP_INFO(this->get_logger(), "ROTATING");
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
            RCLCPP_INFO(this->get_logger(), "NAVIGATING TO POINT");
            if (!_navigationController._endNodeReached)
            {
                RCLCPP_INFO(this->get_logger(), "SET  WHEEL CMD");
                _targetWheelCmd = _lidarNavigation.computeWheelCommands(_pointCloudMsg);
                this->computeVector();
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "POINT REACHED");
                auto arucoRequest = std::make_shared<rover_msgs::srv::ArucoDetection::Request>();
                arucoRequest->command = rover_msgs::srv::ArucoDetection::Request::START;
                arucoRequest->camera_url = Constants::CameraInfo::CAMERA_URL_MAP.at(("Main"));  // TODO Make better
                goalRequested = false;
                goalReached = true;
                _state = eState::DETECTING_ARUCO;
            }
            break;
        case (eState::DETECTING_ARUCO):
            RCLCPP_INFO(this->get_logger(), "GOAL REACHED");
            if (!this->_arucoDetected)
            {
                _targetWheelCmd = _navigationController.rotate();
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "ARUCO DETECTED");
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