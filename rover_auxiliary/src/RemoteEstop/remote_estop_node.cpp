#include "remote_estop_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RemoteStop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

RemoteStop::RemoteStop():
    Node("estop_node")
{
    this->_clientDriveTrainArb = this->create_client<rover_msgs::srv::DriveTrainArbitration>(SERVICE_ARBITRATION_CONTROL);
    this->_srvRemoteEstop = this->create_service<rover_msgs::srv::EmergencyStop>(
        SERVICE_ESTOP,
        [this](const std::shared_ptr<rover_msgs::srv::EmergencyStop::Request> request_,
               std::shared_ptr<rover_msgs::srv::EmergencyStop::Response> response_)
        {
            this->CB_webServer(*request_, *response_);
        });
}

void RemoteStop::CB_webServer(const rover_msgs::srv::EmergencyStop::Request& request_,
                              rover_msgs::srv::EmergencyStop::Response& response_)
{
    auto request = std::make_shared<rover_msgs::srv::DriveTrainArbitration::Request>();
    
    if (request_.request_stop == true)
    {
        request->target_arbitration.arbitration = rover_msgs::msg::DrivetrainArbitration::NONE;
        this->_clientDriveTrainArb->async_send_request(request);
        response_.success = true;
    }
    else if (request_.request_reset == true)
    {
        request->target_arbitration.arbitration = rover_msgs::msg::DrivetrainArbitration::TELEOP;
        this->_clientDriveTrainArb->async_send_request(request);
        response_.success = true;
    }
    else
    {
        response_.success = false;
    }
}