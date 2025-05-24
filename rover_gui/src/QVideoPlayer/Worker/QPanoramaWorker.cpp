#include "QPanoramaWorker.hpp"

QPanoramaWorker::QPanoramaWorker(bool start_, QObject* parent_):
    QWorker(start_, parent_)
{
}

QPanoramaWorker::~QPanoramaWorker()
{
    this->finish();
}

void QPanoramaWorker::takePanoramaManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::PhotoPanoramique>> client_panoramique_,
                                          std::string cameraUrl_,
                                          uint16_t playerIndex_,
                                          std::string basePath_,
                                          uint16_t duration_)
{
    this->addTask(
        [this, client_panoramique_, cameraUrl_, playerIndex_, basePath_, duration_](void)
        {
            this->takePanoramaInternal(client_panoramique_, cameraUrl_, playerIndex_, basePath_, duration_);
        });
}

void QPanoramaWorker::takePanoramaInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::PhotoPanoramique>> client_panoramique_,
                                           std::string cameraUrl_,
                                           uint16_t playerIndex_,
                                           std::string basePath_,
                                           uint16_t duration_)
{
    std::string status;

    std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Request> request
        = std::make_shared<rover_msgs::srv::PhotoPanoramique::Request>();

    request->camera_url = cameraUrl_;
    request->base_path = basePath_ + "/camera";

    if (!client_panoramique_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "ERROR: couldn't take panorama \npanorama client is invalid");
        return;
    }

    rclcpp::Client<rover_msgs::srv::PhotoPanoramique>::FutureAndRequestId future_and_request
        = client_panoramique_->async_send_request(request);

    std::future<std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Response>> future_result
        = std::move(future_and_request.future);

    emit this->PanoramaStarted(duration_, playerIndex_);
}