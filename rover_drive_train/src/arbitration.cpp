#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"

#include "rover_msgs/srv/drive_train_arbitration.hpp"

//Class definition
class Arbitration : public rclcpp::Node
{
    public:
        Arbitration();
    private:
        rclcpp::Subscription<rover_msgs::msg::PropulsionMotor>::SharedPtr _sub_motor_cmd;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _sub_base_heartbeat;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _sub_rover_heartbeat;

        rclcpp::Publisher<rover_msgs::msg::PropulsionMotor>::SharedPtr _pub_abtr;

        rclcpp::Service<rover_msgs::srv::DriveTrainArbitration>::SharedPtr _srv_control_demux;

        void callbackDemux(const std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Request> request,
                                std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Response> response);
};

Arbitration::Arbitration() : Node("arbitration")
{
    _sub_base_heartbeat = this->create_subscription<std_msgs::msg::Empty>("/base/heartbeat",
                                                                                    1,
                                                                                    std::bind(&Arbitration::callbackDemux, this, std::placeholders::_1));
    _sub_rover_heartbeat = this->create_subscription<std_msgs::msg::Empty>("/rover/heartbeat",
                                                                                    1,
                                                                                    std::bind(&Arbitration::callbackDemux, this, std::placeholders::_1));
    _sub_motor_cmd = this->create_subscription<rover_msgs::msg::PropulsionMotor>("/rover/drive_train/cmd/in/security",
                                                                                    1,
                                                                                    std::bind(&Arbitration::callbackDemux, this, std::placeholders::_1));

    _pub_abtr = this->create_publisher<rover_msgs::msg::PropulsionMotor>("/rover/drive_train/cmd/out/raw", 1);

    _srv_control_demux = this->create_service<rover_msgs::srv::DriveTrainArbitration>("demux_control_cmd", std::bind(&Arbitration::callbackDemux, this, std::placeholders::_1, std::placeholders::_2));
}

void Arbitration::callbackDemux(const std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Request> request,
                                      std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Response> response)
{
  
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Arbitration>());
    rclcpp::shutdown();
    
    return 0;
}
