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

void QStatusWorker::requestDeviceStatusInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::Empty>> client_requestErrorStatus_)
{
    std::shared_ptr<rover_msgs::srv::Empty::Request> request = std::make_shared<rover_msgs::srv::Empty::Request>();

    if (!client_requestErrorStatus_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "ERROR: couldn't ask for status \nrequest error status client is invalid");
        return;
    }

    auto result_future = client_requestErrorStatus_->async_send_request(
        request,
        [this](rclcpp::Client<rover_msgs::srv::Empty>::SharedFuture future)
        {
            auto response = future.get();
            if (response->success)
            {
                emit this->onRequestDeviceStatusSuccessful(response->success, response->message);
            }
        });
}