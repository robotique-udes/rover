#include "QPlayerWorker.hpp"

#include <QDebug>
#include <rclcpp/rclcpp.hpp>


QPlayerWorker::QPlayerWorker(bool start_, QObject* parent_):
    QWorker(start_, parent_)
{

}


QPlayerWorker::~QPlayerWorker()
{
    this->finish();
}

void QPlayerWorker::manageDetectionInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> 
    client_ArucoDetectionManager_, std::string _camURL, bool start_)
    {
        auto request = std::make_shared<rover_msgs::srv::ArucoDetection::Request>();
        
        if(start_)
        {
            request->command = rover_msgs::srv::ArucoDetection::Request::START;
        }
        else
        {
           request->command = rover_msgs::srv::ArucoDetection::Request::STOP; 
        }
        request->camera_url = _camURL;

        auto result = client_ArucoDetectionManager_->async_send_request(request);


        while (rclcpp::ok() && result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
        // Continue working, no need to spin the node manually, as it's handled elsewhere
        }

        // Check if the result was successful
        if (result.get() != nullptr) {
            emit detectionHandledSuccessfully(true);  // Notify widget of success
        } else {
            emit detectionHandledSuccessfully(false);  // Notify widget of failure
        }
    }

void QPlayerWorker::manageDetection(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> 
    client_ArucoDetectionManager_, std::string _camURL, bool start_)
{
    this->addTask(
        [this,client_ArucoDetectionManager_, _camURL, start_](void)
        {
            this->manageDetectionInternal(client_ArucoDetectionManager_, _camURL, start_);
        });
}

void QPlayerWorker::updateDetectionManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager,std::string camURL_)
{
    this->addTask(
            [this, client_ArucoDetectionManager,camURL_](void)
            {
                this->updateDetectionInternal(client_ArucoDetectionManager,camURL_);
            });
}

void QPlayerWorker::updateDetectionInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager_,std::string camURL_)
{
    auto request = std::make_shared<rover_msgs::srv::ArucoDetection::Request>();
  
    request->command = rover_msgs::srv::ArucoDetection::Request::INFO;

    request->camera_url = "";

    auto result = client_ArucoDetectionManager_->async_send_request(request);


    while (rclcpp::ok() && result.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready) {
    // Continue working, no need to spin the node manually, as it's handled elsewhere
    }
    bool urlFound = false;
    if (result.valid()) 
    {
        std::shared_ptr<rover_msgs::srv::ArucoDetection::Response> response = result.get();  // Capture the response data

        if(response!=nullptr)
        {
            std::vector<std::string> liveURLs = response->urls;

            for(const auto& url:liveURLs)
            {
                if (camURL_== url)
                {
                    urlFound = true;
                    break;
                }
            }
        }
    }

    emit urlFoundInDetection(urlFound);  // Notify widget of failure
}

