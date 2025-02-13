#include "aruco_detection_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoDetectionNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    rclcpp::shutdown();
    return 0;
}

ArucoDetectionNode::ArucoDetectionNode(void): Node("aruco_detection_node")
{
    declare_parameter("DEBUG_MODE", false);  // Command-line argument for quick debugging
    get_parameter("DEBUG_MODE", DEBUG_MODE);

    _publisher = this->create_publisher<rover_msgs::msg::Aruco>("aruco_detection", 10);
    _timerPublisher
        = this->create_wall_timer(std::chrono::milliseconds(DELAY_PUBLISHER_MS), [this]() { this->CB_aruco_publisher(); });

    _timerDetection
        = this->create_wall_timer(std::chrono::milliseconds(DELAY_DETECTION_MS), [this]() { this->CB_aruco_detection(); });

    _detection = std::make_unique<Detection>("v4l2:///dev/video0");
}

void ArucoDetectionNode::CB_aruco_publisher(void)
{
    std::vector<uint16_t> detectedArucos;

    {
        std::lock_guard<std::mutex> lock(_mutex);

        detectedArucos = _detection->getValidatedIds();
    }

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

void ArucoDetectionNode::CB_aruco_detection(void)
{
    std::lock_guard<std::mutex> lock(_mutex);

    _detection->update(DEBUG_MODE);
}