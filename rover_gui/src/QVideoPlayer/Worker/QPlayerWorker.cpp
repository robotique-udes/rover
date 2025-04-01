#include "QPlayerWorker.hpp"

#include <QDebug>
#include <rclcpp/rclcpp.hpp>


QPlayerWorker::QPlayerWorker(bool start_, QObject* parent_):
    QWorker(start_, parent_)
{
    _timer_serviceCall= RoverLib::Timer<uint64_t, RoverLib::millis>(MAX_DELAY_SERVICE_CALL);
}


QPlayerWorker::~QPlayerWorker()
{
    this->finish();
}

void QPlayerWorker::manageDetectionInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> 
    client_ArucoDetectionManager_, std::string _camURL, bool start_)
    {
        auto request = std::make_shared<rover_msgs::srv::ArucoDetection::Request>();
        qDebug()<<"internal";

        if(start_)
        {
            qDebug()<<"sending request";
            request->command = rover_msgs::srv::ArucoDetection::Request::START;
        }
        else
        {
           request->command = rover_msgs::srv::ArucoDetection::Request::STOP; 
        }
        request->camera_url = _camURL;

        auto result = client_ArucoDetectionManager_->async_send_request(request);

        _timer_serviceCall.reset();
        while (rclcpp::ok() && result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) 
        {
            qDebug()<<"waitinggg";
            if(_timer_serviceCall.isDone())
            {
                qDebug()<<"timeout starting stream";
                break;
            }
        }

        if(result.valid())
        {
            if (result.get() != nullptr) {
                emit detectionHandledSuccessfully(true);  // Notify widget of success
            } else {
                emit detectionHandledSuccessfully(false);  // Notify widget of failure
            }
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

void QPlayerWorker::updateDetectionManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager)
{
    qDebug()<<"adding task";
    this->addTask(
            [this, client_ArucoDetectionManager](void)
            {
                this->updateDetectionInternal(client_ArucoDetectionManager);
            });
}

void QPlayerWorker::updateDetectionInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager_)
{
    qDebug()<<"internal update";
    auto request = std::make_shared<rover_msgs::srv::ArucoDetection::Request>();
  
    request->command = rover_msgs::srv::ArucoDetection::Request::INFO;

    request->camera_url = "";

    auto result = client_ArucoDetectionManager_->async_send_request(request);


    _timer_serviceCall.reset();

    bool service_call_interrupte = false;
    
    while (rclcpp::ok() && result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) 
    {
        if(_timer_serviceCall.isDone())
        {
            service_call_interrupte = true;
            break;
        }
    }

    std::vector<std::string> liveURLs;

    if (result.valid()) 
    {        
            if(!service_call_interrupte)
            {
            std::shared_ptr<rover_msgs::srv::ArucoDetection::Response> response = result.get();  // Capture the response data


            if(response!=nullptr)
            {
                liveURLs = response->urls;


            }
            }

    }
    emit urlFoundInDetection(liveURLs);  // Notify widget of failure

}

