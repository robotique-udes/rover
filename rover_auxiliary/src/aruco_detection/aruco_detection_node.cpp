#include "aruco_detection_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectionNode>());
    rclcpp::shutdown();
    return 0;
}

ArucoDetectionNode::ArucoDetectionNode(void): Node("aruco_detection_node")
{
    _publisher = this->create_publisher<rover_msgs::msg::Aruco>("detected_arucos", 10);
    _timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ArucoDetectionNode::CB_aruco, this));
    _detection = std::make_unique<Detection>("v4l2:///dev/video0");
}

ArucoDetectionNode::~ArucoDetectionNode(void) {}

void ArucoDetectionNode::CB_aruco(void)
{
    std::vector<uint16_t> detectedArucos = _detection->update(DEBUG_MODE);

    rover_msgs::msg::Aruco msg;

    for (auto id : detectedArucos)
    {
        msg.id.push_back(static_cast<uint8_t>(id));  // Ensure ID is cast to uint8
    }

    _publisher->publish(msg);

    if (DEBUG_MODE)
    {
        if (!detectedArucos.empty())
        {
            std::string marker_list = "Publishing detected Aruco markers: ";
            for (auto id : detectedArucos)
            {
                marker_list += std::to_string(id) + " ";
            }
            RCLCPP_INFO(this->get_logger(), "%s", marker_list.c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No Aruco markers detected to publish");
        }
    }
}
