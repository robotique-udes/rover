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
    RCLCPP_INFO(this->get_logger(), "Received desired GPS position: lat=%.6f, lon=%.6f",
             request_->desired_latitude, request_->desired_longitude);

    std::array<float, TO_UNDERLYING(NavigationController::eGpsData::eLAST)> desiredGpsData = {
        request_->desired_latitude,
        request_->desired_longitude,
        0.0F  // Heading is not used in this context
    };

    goalReached = false;
    goalRequested = true;

    // TODO : Add check
    response_->success = true;

    _navigationController.getDesiredGpsData(desiredGpsData);
}

void GoalManager::driveTrainPublisher(void)
{
    if (goalRequested && !goalReached)
    {
        RCLCPP_INFO(this->get_logger(), "Processing goal request...");
        NavigationController::eRotationDirection rotationDirection = _navigationController.computeRotationDirection();

        rover_msgs::msg::PropulsionMotor wheelCmdMsg;

        switch (rotationDirection)
        {
            case NavigationController::eRotationDirection::CLOCKWISE:
                RCLCPP_INFO(this->get_logger(), "Rotating clockwise");
                wheelCmdMsg.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_FRONT_LEFT]
                    = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
                wheelCmdMsg.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_REAR_LEFT]
                    = -Constants::DriveTrain::SPEED_FACTOR_NORMAL;
                wheelCmdMsg.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_FRONT_RIGHT]
                    = Constants::DriveTrain::SPEED_FACTOR_NORMAL * -1.0F;
                wheelCmdMsg.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_REAR_RIGHT]
                    = -Constants::DriveTrain::SPEED_FACTOR_NORMAL * -1.0F;
                break;
            case NavigationController::eRotationDirection::COUNTERCLOCKWISE:
                RCLCPP_INFO(this->get_logger(), "Rotating counterclockwise");
                wheelCmdMsg.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_FRONT_LEFT]
                    = Constants::DriveTrain::SPEED_FACTOR_NORMAL * -1.0F;
                wheelCmdMsg.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_REAR_LEFT]
                    = -Constants::DriveTrain::SPEED_FACTOR_NORMAL * -1.0F;
                wheelCmdMsg.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_FRONT_RIGHT]
                    = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
                wheelCmdMsg.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_REAR_RIGHT]
                    = -Constants::DriveTrain::SPEED_FACTOR_NORMAL;
                break;
            case NavigationController::eRotationDirection::NO_ROTATION:
                RCLCPP_INFO(this->get_logger(), "Heading reached");
                goalReached = true;
            default:
                break;
        }

        _pub_auto_cmd->publish(wheelCmdMsg);
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalManager>());
    rclcpp::shutdown();
}