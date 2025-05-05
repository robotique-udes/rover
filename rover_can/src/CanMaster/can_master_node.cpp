#include "can_master_node.hpp"

int main(int argc_, char* argv_[])
{
    rclcpp::init(argc_, argv_);
    rclcpp::spin(std::make_shared<CanMasterNode>());
    rclcpp::shutdown();

    return 0;
}

CanMasterNode::CanMasterNode():
    Node("CanMasterNode")
{
    _pub_CanDeviceErrorState = this->create_publisher<rover_msgs::msg::CanDeviceStatus>(CAN_DEVICE_STATUS_TOPIC, 1);
    _srv_canDeviceErrorStateRequest = this->create_service<rover_msgs::srv::Empty>(
        ERROR_STATE_SRV_NAME,
        [this](rover_msgs::srv::Empty::Request::SharedPtr request_, rover_msgs::srv::Empty::Response::SharedPtr response_)
        {
            this->CB_ROS_canDeviceErrorStateRequest(request_, response_);
        });

    _timerUpdateCan = this->create_wall_timer(std::chrono::milliseconds(1),
                                              [this](void)
                                              {
                                                  this->CB_updateCan();
                                              });
}

void CanMasterNode::CB_updateCan(void)
{
    if (!_nodeAttachedToDevices)
    {
        for (auto& device : _deviceArray)
        {
            if (device)
            {
                device->attachNode(this->shared_from_this());
            }
        }
        _nodeAttachedToDevices = true;
    }

    _canManager.update();
}

void CanMasterNode::CB_ROS_canDeviceErrorStateRequest(rover_msgs::srv::Empty::Request::SharedPtr,
                                                      rover_msgs::srv::Empty::Response::SharedPtr response_)
{
    bool success = _canManager.sendErrorStateRequest();
    if (!response_)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Received srv call with nullptr response. Request is still sent but response won't be populated");
        return;
    }

    response_->success = success;
    response_->message = std::string("ErrorState request succesfully sent on CanBus network, response from all devices can "
                                     "be retrieved on /rover/can/devices_status topic");
}

void CanMasterNode::CB_CAN_errorStateRecv(RoverCan2::Constant::eDeviceId deviceId_, const RoverCan2::Msgs::ErrorState& canMsg_)
{
    rover_msgs::msg::CanDeviceStatus rosMsg;
    rosMsg.id = TO_UNDERLYING(deviceId_);

    if (canMsg_.getData().error)
    {
        rosMsg.error_state = rover_msgs::msg::CanDeviceStatus::STATUS_ERROR;
    }
    else
    {
        rosMsg.error_state = rover_msgs::msg::CanDeviceStatus::STATUS_OK;
    }

    _pub_CanDeviceErrorState->publish(rosMsg);
}
