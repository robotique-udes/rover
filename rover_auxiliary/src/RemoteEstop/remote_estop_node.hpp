#ifndef __ESTOP_NODE_HPP__
#define __ESTOP_NODE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/srv/emergency_stop.hpp>
#include <rover_msgs/srv/drive_train_arbitration.hpp>
#include <rover_msgs/msg/drivetrain_arbitration.hpp>

class RemoteStop : public rclcpp::Node
{
    static constexpr const char* SERVICE_ESTOP = "/rover/remote_estop/estop_srv";
    static constexpr const char* SERVICE_ARBITRATION_CONTROL = "/rover/drive_train/demux_control";

  private:
    rclcpp::Service<rover_msgs::srv::EmergencyStop>::SharedPtr _srvRemoteEstop;
    rclcpp::Client<rover_msgs::srv::DriveTrainArbitration>::SharedPtr _clientDriveTrainArb;

  public:
    RemoteStop();

  private:
    void CB_webServer(const rover_msgs::srv::EmergencyStop::Request& request_,
                      rover_msgs::srv::EmergencyStop::Response& response_);
};

#endif