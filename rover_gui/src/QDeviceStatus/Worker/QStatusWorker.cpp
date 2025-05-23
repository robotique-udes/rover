#include "QStatusWorker.hpp"

QStatusWorker::QStatusWorker(bool start_, QObject* parent_):
    QWorker(start_, parent_)
{
}

QStatusWorker::~QStatusWorker()
{
    this->finish();
}

void QStatusWorker::requestDeviceStatusManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::Empty>> client_requestErrorStatus_)
{
    this->addTask(
        [this, client_requestErrorStatus_](void)
        {
            this->requestDeviceStatusInternal(client_requestErrorStatus_);
        });
}

void QStatusWorker::requestDeviceStatusInternal(
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::Empty>> client_requestErrorStatus_)
{
    bool success = false;
    std::string status;

    std::shared_ptr<rover_msgs::srv::Empty::Request> request = std::make_shared<rover_msgs::srv::Empty::Request>();

    if (!client_requestErrorStatus_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "ERROR: couldn't ask for status \nrequest error status client is invalid");
        return;
    }

    rclcpp::Client<rover_msgs::srv::Empty>::FutureAndRequestId future_and_request
        = client_requestErrorStatus_->async_send_request(request);

    std::future<std::shared_ptr<rover_msgs::srv::Empty::Response>> future_result = std::move(future_and_request.future);
    if (future_result.valid())
    {
        std::shared_ptr<rover_msgs::srv::Empty::Response> response = future_result.get();

        if (response != nullptr)
        {
            success = response->success;
            status = response->message;
        }

        emit this->onRequestDeviceStatusSuccessful(success, status);
    }
}