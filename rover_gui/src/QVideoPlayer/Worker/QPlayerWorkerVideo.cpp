#include "QPlayerWorkerVideo.hpp"

QPlayerWorkerVideo::QPlayerWorkerVideo(bool start_, QObject* parent_):
    QWorker(start_, parent_),
    _timer_serviceCallCamera(MAX_DELAY_SERVICE_CALL)
{
}

QPlayerWorkerVideo::~QPlayerWorkerVideo()
{
    this->finish();
}

void QPlayerWorkerVideo::takeScreenshotManager(
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
    std::string camera_URL_,
    uint16_t tag_)
{
    RCLCPP_INFO(rclcpp::get_logger("GUI"), "Manager receveived");
    this->addTask(
        [this, client_CameraControl_, camera_URL_, tag_](void)
        {
            takeScreenshotInternal(client_CameraControl_, camera_URL_, tag_);
        });
}

void QPlayerWorkerVideo::takeScreenshotInternal(
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
    std::string camera_URL_,
    uint16_t tag_)
{
    RCLCPP_INFO(rclcpp::get_logger("GUI"), "Internal received");
    bool success = false;
    std::string status;

    auto request = std::make_shared<rover_msgs::srv::CameraControl::Request>();
    request->command = rover_msgs::srv::CameraControl::Request::TAKE_PICTURE;
    request->camera_url = camera_URL_;

    if (!client_CameraControl_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "ERROR: couldn't take screenshot \ncamera control client is invalid");
        return;
    }

    auto result = client_CameraControl_->async_send_request(request);

    _timer_serviceCallCamera.reset();

    bool service_call_interrupt = false;

    while (rclcpp::ok() && result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
    {
        if (_timer_serviceCallCamera.isReady())
        {
            service_call_interrupt = true;
            break;
        }
    }

    if (result.valid())
    {
        if (!service_call_interrupt)
        {
            std::shared_ptr<rover_msgs::srv::CameraControl::Response> response = result.get();

            if (response != nullptr)
            {
                success = response->success;
                status = response->status;
            }
        }
    }

    emit screenshotHandledSuccessfully(success, status, tag_);
}

void QPlayerWorkerVideo::startRecordingManager(
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
    std::string camera_URL_,
    uint16_t tag_)
{
    this->addTask(
        [this, client_CameraControl_, camera_URL_, tag_](void)
        {
            startRecordingInternal(client_CameraControl_, camera_URL_, tag_);
        });
}

void QPlayerWorkerVideo::stopRecordingManager(
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
    std::string camera_URL_,
    uint16_t tag_)
{
    this->addTask(
        [this, client_CameraControl_, camera_URL_, tag_](void)
        {
            stopRecordingInternal(client_CameraControl_, camera_URL_, tag_);
        });
}

void QPlayerWorkerVideo::startRecordingInternal(
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
    std::string camera_URL_,
    uint16_t tag_)
{
    bool success = false;
    std::string status;

    auto request = std::make_shared<rover_msgs::srv::CameraControl::Request>();

    request->command = rover_msgs::srv::CameraControl::Request::START_RECORDING;

    request->camera_url = camera_URL_;

    if (!client_CameraControl_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "ERROR: couldn't take screenshot \ncamera control client is invalid");
        return;
    }

    auto result = client_CameraControl_->async_send_request(request);

    _timer_serviceCallCamera.reset();

    bool service_call_interrupt = false;

    while (rclcpp::ok() && result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
    {
        if (_timer_serviceCallCamera.isReady())
        {
            service_call_interrupt = true;
            break;
        }
    }

    if (result.valid())
    {
        if (!service_call_interrupt)
        {
            std::shared_ptr<rover_msgs::srv::CameraControl::Response> response = result.get();

            if (response != nullptr)
            {
                success = response->success;
                status = response->status;
            }
        }
    }

    emit startRecordingHandledSuccessfully(success, status, tag_);
}

void QPlayerWorkerVideo::stopRecordingInternal(
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
    std::string camera_URL_,
    uint16_t tag_)
{
    bool success = false;
    std::string status;

    auto request = std::make_shared<rover_msgs::srv::CameraControl::Request>();

    request->command = rover_msgs::srv::CameraControl::Request::STOP_RECORDING;

    request->camera_url = camera_URL_;

    if (!client_CameraControl_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "ERROR: couldn't take screenshot \ncamera control client is invalid");
        return;
    }

    auto result = client_CameraControl_->async_send_request(request);

    _timer_serviceCallCamera.reset();

    bool service_call_interrupt = false;

    while (rclcpp::ok() && result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
    {
        if (_timer_serviceCallCamera.isReady())
        {
            service_call_interrupt = true;
            break;
        }
    }

    if (result.valid())
    {
        if (!service_call_interrupt)
        {
            std::shared_ptr<rover_msgs::srv::CameraControl::Response> response = result.get();

            if (response != nullptr)
            {
                success = response->success;
                status = response->status;
            }
        }
    }

    emit stopRecordingHandledSuccessfully(success, status, tag_);
    return;
}