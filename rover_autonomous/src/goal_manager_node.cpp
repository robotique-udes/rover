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

    _navigationController.headingBuffer_ = HEADING_BUFFER;  // TODO make this cleaner
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

bool GoalManager::desiredHeadingReached(NavigationController::eRotationDirection rotationDirection_)
{
    rover_msgs::msg::PropulsionMotor wheelCmdMsg;

    bool desiredHeadingReached = false;

    switch (rotationDirection_)
    {
        case NavigationController::eRotationDirection::CLOCKWISE:
            RCLCPP_INFO(this->get_logger(), "Rotating clockwise");
            wheelCmdMsg.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_FRONT_LEFT]
                = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
            wheelCmdMsg.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_REAR_LEFT]
                = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
            wheelCmdMsg.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_FRONT_RIGHT]
                = Constants::DriveTrain::SPEED_FACTOR_NORMAL * -1.0F;
            wheelCmdMsg.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_REAR_RIGHT]
                = Constants::DriveTrain::SPEED_FACTOR_NORMAL * -1.0F;
            break;
        case NavigationController::eRotationDirection::COUNTERCLOCKWISE:
            RCLCPP_INFO(this->get_logger(), "Rotating counterclockwise");
            wheelCmdMsg.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_FRONT_LEFT]
                = Constants::DriveTrain::SPEED_FACTOR_NORMAL * -1.0F;
            wheelCmdMsg.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_REAR_LEFT]
                = Constants::DriveTrain::SPEED_FACTOR_NORMAL * -1.0F;
            wheelCmdMsg.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_FRONT_RIGHT]
                = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
            wheelCmdMsg.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_REAR_RIGHT]
                = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
            break;
        case NavigationController::eRotationDirection::NO_ROTATION:
            desiredHeadingReached = true;
        default:
            break;
    }

    _pub_auto_cmd->publish(wheelCmdMsg);
    return desiredHeadingReached;
}

void GoalManager::driveTrainPublisher(void)
{
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
                _state = eState::NAVIGATING_TO_POINT;
            }
            break;
        case (eState::NAVIGATING_TO_POINT):
        {
            RCLCPP_INFO(this->get_logger(), "HEADING REACHED");
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