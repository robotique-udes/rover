#include "goal_manager_node.hpp"
#include "rover_lib2/helpers/constants.hpp"

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
    _srv_desiredGps = this->create_service<rover_msgs::srv::DesiredGpsPosition>(
        SRV_GOAL_NAME,
        [this](const rover_msgs::srv::DesiredGpsPosition::Request::SharedPtr request_,
               rover_msgs::srv::DesiredGpsPosition::Response::SharedPtr response_)
        {
            this->CB_desiredGps(request_, response_);
        });
    _pub_auto_cmd = this->create_publisher<rover_msgs::msg::PropulsionMotor>(TOPIC_WHEEL_CMD_NAME, QOS_DEFAULT);
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
    if(arucoMsg_.valid)
    {
        RCLCPP_INFO(this->get_logger(), "Detected aruco => %d", arucoMsg_.id);
        this->_arucoDetected = true;
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

void GoalManager::driveTrainPublisher(void)
{
    // TODO Change if logic -> switch sides
    switch (_state)
    {
        case (eState::IDLE):
            RCLCPP_INFO(this->get_logger(), "IDLE");
            if (!goalRequested)
            {
                this->_targetWheelCmd = _navigationController.idleCmd();
            }
            else
            {
                _state = eState::ROTATING;
            }
            break;
        case (eState::ROTATING):
            RCLCPP_INFO(this->get_logger(), "ROTATING");
            if (!_navigationController._desiredHeadingReached)
            {
                this->_targetWheelCmd = _navigationController.getToHeading();
            }
            else
            {
                // _state = eState::NAVIGATING_TO_POINT;
                _state = eState::IDLE;
            }
            break;
        case (eState::NAVIGATING_TO_POINT):
            RCLCPP_INFO(this->get_logger(), "NAVIGATING TO POINT");
            if (!_navigationController._endNodeReached)
            {
                _targetWheelCmd = _navigationController.setWheelCmd();
                goalRequested = false;
                goalReached = true;
            }
            else
            {
                auto arucoRequest = std::make_shared<rover_msgs::srv::ArucoDetection::Request>();
                arucoRequest->command = rover_msgs::srv::ArucoDetection::Request::START;
                arucoRequest->camera_url = Constants::CameraInfo::CAMERA_URL_MAP.at(("Main"));  // TODO Make better
                _state = eState::DETECTING_ARUCO;
            }
        case (eState::DETECTING_ARUCO):
            RCLCPP_INFO(this->get_logger(), "HEADING REACHED");
            if (!this->_arucoDetected)
            {
                _targetWheelCmd = _navigationController.rotate();
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "ARUCO DETECTED");
                _targetWheelCmd = _navigationController.idleCmd();
                _state = eState::IDLE;
                // TODO END NODE REACHED
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