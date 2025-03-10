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

    _srv_detectionManager = this->create_service<rover_msgs::srv::ArucoDetection>(
        "/rover/auxiliary/aruco/manager",
        [this](const std::shared_ptr<rover_msgs::srv::ArucoDetection::Request> request_,
               std::shared_ptr<rover_msgs::srv::ArucoDetection::Response> response_) { this->CB_srv(request_, response_); });
}

void ArucoDetectionNode::getParams(int argc, char** argv)
{
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
    std::vector<uint16_t> tempIds;

    for (const auto& it : _detections)
    {
        {
            std::lock_guard<std::mutex> lock(_detectedArucosMutex);
            tempIds = it.second.getValidatedIds();
        }
        detectedArucos.insert(detectedArucos.end(), tempIds.begin(), tempIds.end());  // fix doublons
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

    for (auto it = _detections.begin(); it != _detections.end();)
    {
        if (it->second.getErrorFrameCount() > ALLOWED_ERROR_FRAME)
        {
            RCLCPP_WARN(this->get_logger(), "Detection at %s has been shutdown", it->second.getCamURL().c_str());
            it = _detections.erase(it);
            _nbrOngoingDetection--;
        }
        else
        {
            it->second.update(_debugMode);
            ++it;
        }
    }
}

void ArucoDetectionNode::CB_srv(const std::shared_ptr<rover_msgs::srv::ArucoDetection::Request> request_,
                                std::shared_ptr<rover_msgs::srv::ArucoDetection::Response> response_)
{
    response_->success = true;

    if (request_->start)
    {
        response_->success = this->startDetection(request_->camera_url);
    }

    if (request_->stop)
    {
        response_->success = this->stopDetection(request_->camera_url);
    }

    response_->nbr_ongoing_streams = _nbrOngoingDetection;

    if (request_->info)
    {
        response_->info = this->infoDetection();
    }
}

bool ArucoDetectionNode::startDetection(std::string URL_)
{
    std::lock_guard<std::mutex> lock(_detectedArucosMutex);
    if (!_detections.emplace(URL_, Detection(URL_, _nbrOngoingDetection + 1)).second)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to start detection at %s", URL_.c_str());
        return false;
    }

    _nbrOngoingDetection++;
    return true;
}

bool ArucoDetectionNode::stopDetection(std::string URL_)
{
    std::lock_guard<std::mutex> lock(_detectedArucosMutex);

    if (!_detections.erase(URL_))
    {
        RCLCPP_WARN(this->get_logger(), "Failed to stop detection at %s", URL_.c_str());
        return false;
    }

    _nbrOngoingDetection--;
    return true;
}

std::string ArucoDetectionNode::infoDetection(void)
{
    std::lock_guard<std::mutex> lock(_detectedArucosMutex);
    std::string message = "";

    for (auto& it : _detections)
    {
        message = message + std::to_string(it.second.getTag()) + " : " + it.second.getCamURL() + "   ";
    }
    return message;
}
