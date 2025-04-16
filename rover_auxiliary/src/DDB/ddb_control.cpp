#include "ddb_control_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DDBControlNode>();
  //rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

DDBControlNode::DDBControlNode(): Node("ddb_control")
{
  _srv_control = this->create_service<rover_msgs::srv::DDBControl>(
    "/rover/auxiliary/ddb_control",
    [this](const std::shared_ptr<rover_msgs::srv::DDBControl::Request> request_,
           std::shared_ptr<rover_msgs::srv::DDBControl::Response> response_)
    {
        if (!request_ || !response_)
        {
            RCLCPP_ERROR(this->get_logger(), "NULL request or response received.");
            return;
        }
        this->ddbControl(*request_, *response_);
    });

}

void DDBControlNode::ddbControl(const rover_msgs::srv::DDBControl::Request& request_,
                                rover_msgs::srv::DDBControl::Response& response_)
{
  printf("Test");
}