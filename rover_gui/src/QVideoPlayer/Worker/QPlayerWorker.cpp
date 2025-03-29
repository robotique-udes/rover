#include "QPlayerWorker.hpp"

#include <rclcpp/rclcpp.hpp>

QPlayerWorker::QPlayerWorker(bool start_, QObject* parent_):
    QWorker(start_, parent_)
{
}

QPlayerWorker::~QPlayerWorker()
{
    this->finish();
}

void QPlayerWorker::startDetection(std::shared_ptr<rclcpp::Node> node_, std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> 
    client_ArucoDetectionManager_, std::string _camURL)
    {
        auto request = std::make_shared<rover_msgs::srv::ArucoDetection::Request>();
        request->command = rover_msgs::srv::ArucoDetection::Request::START;
        request->camera_url = _camURL;

        auto result = client_ArucoDetectionManager_->async_send_request(request);


        while (rclcpp::ok() && result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
        // Continue working, no need to spin the node manually, as it's handled elsewhere
        }

        // Check if the result was successful
        if (result.get() != nullptr) {
            emit detectionStartedSuccessfully(true);  // Notify widget of success
        } else {
            emit detectionStartedSuccessfully(false);  // Notify widget of failure
        }
    }