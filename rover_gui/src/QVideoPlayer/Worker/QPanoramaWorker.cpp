#include "QPanoramaWorker.hpp"

QPanoramaWorker::QPanoramaWorker(bool start_, QObject* parent_):
    QWorker(start_, parent_)
{
}

QPanoramaWorker::~QPanoramaWorker()
{
    this->finish();
}

void QPanoramaWorker::takePanoramaManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::Panorama>> client_panoramique_,
                                          const std::string& cameraUrl_,
                                          uint16_t playerIndex_,
                                          const std::string& basePath_,
                                          uint16_t duration_)
{
    this->addTask(
        [this, client_panoramique_, &cameraUrl_, playerIndex_, &basePath_, duration_](void)
        {
            this->takePanoramaInternal(client_panoramique_, cameraUrl_, playerIndex_, basePath_, duration_);
        });
}

void QPanoramaWorker::takePanoramaInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::Panorama>> client_panoramique_,
                                           const std::string& cameraUrl_,
                                           uint16_t playerIndex_,
                                           const std::string& basePath_,
                                           uint16_t duration_)
{
    std::shared_ptr<rover_msgs::srv::Panorama::Request> request
        = std::make_shared<rover_msgs::srv::Panorama::Request>();

    request->camera_url = cameraUrl_;
    request->base_path = basePath_ + CAMERA_PATH;
    request->duration = duration_;

    if (!client_panoramique_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "ERROR: couldn't take panorama, panorama client is invalid");
        emit this->panoramaFinished(false, "Panorama client was invalid", playerIndex_);
        return;
    }

    rclcpp::Client<rover_msgs::srv::Panorama>::FutureAndRequestId future_and_request
        = client_panoramique_->async_send_request(request);

    std::future<std::shared_ptr<rover_msgs::srv::Panorama::Response>> future_result
        = std::move(future_and_request.future);

    emit this->panoramaStarted(duration_, playerIndex_);

    if (future_result.wait_for(std::chrono::milliseconds(2U * request->duration)) == std::future_status::ready)
    {
        std::shared_ptr<rover_msgs::srv::Panorama_Response> response = future_result.get();
        emit this->panoramaFinished(response->success, response->status, playerIndex_);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Panorama service call timed out");
        emit this->panoramaFinished(false, "Panorama service call timed out", playerIndex_);
    }
}