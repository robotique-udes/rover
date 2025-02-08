#include "aruco_detection_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectionNode>());
    rclcpp::shutdown();
    return 0;
}

ArucoDetectionNode::ArucoDetectionNode(void): Node("aurco_detection_node")
{
    publisher_ = this->create_publisher<rover_msgs::msg::Aruco>("detected_arucos", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ArucoDetectionNode::ArucoCallback, this));
    detection_ = std::make_unique<ArucoDetection>("v4l2:///dev/video0");
}

ArucoDetectionNode::~ArucoDetectionNode(void) {}

void ArucoDetectionNode::ArucoCallback(void)
{
    std::vector<uint16_t> detectedArucos = detection_->update(DEBUG_MODE);

    rover_msgs::msg::Aruco msg;

    for (auto id : detectedArucos)
    {
        msg.id.push_back(static_cast<uint8_t>(id));  // Ensure ID is cast to uint8
    }

    publisher_->publish(msg);

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
