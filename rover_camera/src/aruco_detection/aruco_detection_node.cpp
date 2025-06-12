#include "aruco_detection_node.hpp"

int main(int argc_, char** argv_)
{
    rclcpp::init(argc_, argv_);
    auto node = std::make_shared<ArucoDetectionNode>(argc_, argv_);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    rclcpp::shutdown();
    return 0;
}

ArucoDetectionNode::ArucoDetectionNode(int argc_, char** argv_):
    Node("aruco_detection_node")
{
    cv::utils::logging::setLogLevel(OPENCV_LOG_LEVEL);

    this->getParams(argc_, argv_);

    _publisher = this->create_publisher<rover_msgs::msg::Aruco>(TOPIC_ARUCO_DETECTED, 10);

    _timer_publisher = this->create_wall_timer(std::chrono::milliseconds(DELAY_PUBLISHER_MS),
                                               [this](void)
                                               {
                                                   this->CB_arucoPublisher();
                                               });

    _timer_detection = this->create_wall_timer(std::chrono::milliseconds(DELAY_DETECTION_MS),
                                               [this](void)
                                               {
                                                   this->CB_arucoDetection();
                                               });

    _srv_detectionManager = this->create_service<rover_msgs::srv::ArucoDetection>(
        SERVICE_SERVER_NAME,
        [this](const std::shared_ptr<rover_msgs::srv::ArucoDetection::Request> request_,
               std::shared_ptr<rover_msgs::srv::ArucoDetection::Response> response_)
        {
            this->CB_srv(request_, response_);
        });
}

void ArucoDetectionNode::getParams(int argc, char** argv)
{
    if (argc > 1 && argv[1][0] == 'd')
    {
        _debugMode = true;
    }
}

void ArucoDetectionNode::CB_arucoPublisher(void)
{
    std::vector<std::vector<uint16_t>> detectedArucos;
    std::vector<std::string> matchingURL;
    std::vector<bool> isValid;
    std::vector<bool> camLost;

    {
        std::lock_guard lock(_detectedArucosMutex);

        for (const auto& [key, detection] : _detections)
        {
            detectedArucos.push_back(detection.getValidatedIds());
            matchingURL.push_back(detection.getCamURL());
            isValid.push_back(detection.isValid());
            camLost.push_back(detection.getCamLost());
        }
    }

    rover_msgs::msg::Aruco msg;

    for (size_t i = 0; i < detectedArucos.size(); ++i)
    {
        const auto& detection = detectedArucos[i];
        const auto& url = matchingURL[i];
        msg.id = detection;
        msg.cam_url = url;
        msg.valid = isValid.at(i);
        if (camLost.at(i))
        {
            msg.valid = false;
        }

        _publisher->publish(msg);

        if (!detection.empty())
        {
            std::string marker_list = "Publishing detected Aruco markers at " + url + " : ";

            for (const auto& id : detection)
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
                RCLCPP_INFO(this->get_logger(), "No Aruco markers detected at %s", url.c_str());
            }
        }
    }
}

void ArucoDetectionNode::CB_arucoDetection(void)
{
    std::lock_guard lock(_detectedArucosMutex);

    for (auto it = _detections.begin(); it != _detections.end();)
    {
        it->second.update(_debugMode);
        ++it;
    }
}

void ArucoDetectionNode::CB_srv(const std::shared_ptr<rover_msgs::srv::ArucoDetection::Request> request_,
                                std::shared_ptr<rover_msgs::srv::ArucoDetection::Response> response_)
{
    response_->success = true;

    if (request_->command == rover_msgs::srv::ArucoDetection::Request::START)
    {
        response_->success = this->startDetection(request_->camera_url);
    }

    else if (request_->command == rover_msgs::srv::ArucoDetection::Request::STOP)
    {
        response_->success = this->stopDetection(request_->camera_url);
    }

    response_->nbr_ongoing_streams = _nbrOngoingDetection;

    infoDetection(response_);
}

bool ArucoDetectionNode::startDetection(const std::string& URL_)
{
    std::lock_guard lock(_detectedArucosMutex);
    if (!_detections.try_emplace(URL_, URL_, _nbrOngoingDetection + 1).second)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to start detection at %s", URL_.c_str());
        return false;
    }

    _nbrOngoingDetection++;
    return true;
}

bool ArucoDetectionNode::stopDetection(const std::string& URL_)
{
    std::lock_guard lock(_detectedArucosMutex);

    if (!_detections.erase(URL_))
    {
        RCLCPP_WARN(this->get_logger(), "Failed to stop detection at %s", URL_.c_str());
        return false;
    }

    _nbrOngoingDetection--;
    return true;
}

void ArucoDetectionNode::infoDetection(std::shared_ptr<rover_msgs::srv::ArucoDetection::Response> response_)
{
    std::lock_guard lock(_detectedArucosMutex);

    if (response_ != nullptr)
    {
        for (auto const& [key, detection] : _detections)
        {
            response_->urls.push_back(detection.getCamURL());
            response_->tags.push_back(detection.getTag());
        }
    }
}
