#include "aruco_detection_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoDetectionNode>(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    rclcpp::shutdown();
    return 0;
}

ArucoDetectionNode::ArucoDetectionNode(int argc, char** argv): Node("aruco_detection_node")
{
    this->getParams(argc, argv);

    _publisher = this->create_publisher<rover_msgs::msg::Aruco>("/rover/video/aruco", 10);
    _timerPublisher
        = this->create_wall_timer(std::chrono::milliseconds(DELAY_PUBLISHER_MS), [this](void) { this->CB_arucoPublisher(); });

    _timerDetection
        = this->create_wall_timer(std::chrono::milliseconds(DELAY_DETECTION_MS), [this](void) { this->CB_arucoDetection(); });

    _detection = std::make_unique<Detection>(_camURL);
}

void ArucoDetectionNode::getParams(int argc, char** argv)
{
    this->declare_parameter<bool>("debug_mode", false);
    this->get_parameter("debug_mode", _debugMode);

    this->declare_parameter<std::string>("default_cam", "rtsp://192.168.144.30:554/1/h264major");
    this->get_parameter("default_cam", _camURL);

    if (argc > 1)
    {
        if (argv[1][0] == 'd')
        {
            _debugMode = true;
        }
    }
}

void ArucoDetectionNode::CB_arucoPublisher(void)
{
    std::vector<uint16_t> detectedArucos;

    if (_detection != nullptr)
    {
        std::lock_guard<std::mutex> lock(_detectedArucosMutex);

        detectedArucos = _detection->getValidatedIds();
    }

    else
    {
        _detection = std::make_unique<Detection>(_camURL);
    }

    rover_msgs::msg::Aruco msg;

    for (const auto& id : detectedArucos)
    {
        msg.id.push_back(id);
    }

    _publisher->publish(msg);

    if (!detectedArucos.empty())
    {
        std::string marker_list = "Publishing detected Aruco markers: ";
        for (auto id : detectedArucos)
        {
            marker_list += std::to_string(id) + " ";
        }
        if (_debugMode)
        {
            RCLCPP_INFO(this->get_logger(), "%s", marker_list.c_str());
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "%s", marker_list.c_str());
        }
    }
    else
    {
        if (_debugMode)
        {
            RCLCPP_INFO(this->get_logger(), "No Aruco markers detected to publish");
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "No Aruco markers detected to publish");
        }
    }
}

void ArucoDetectionNode::CB_arucoDetection(void)
{
    std::lock_guard<std::mutex> lock(_detectedArucosMutex);

    if (_detection != nullptr)
    {
        _detection->update(_debugMode);
    }
}
