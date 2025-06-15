#include "QRecordingWorker.hpp"

QRecordingWorker::QRecordingWorker(bool start_, QObject* parent_):
    QWorker(start_, parent_),
    _timer_serviceCallCamera(MAX_DELAY_SERVICE_CALL)
{
}

QRecordingWorker::~QRecordingWorker() {}

void QRecordingWorker::takeScreenshotManager(
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
    const std::string& cameraUrl_,
    uint16_t playerIndex_,
    const std::string& basePath_)
{
    this->addTask(
        [this, client = client_CameraControl_, cameraUrl = cameraUrl_, playerIndex = playerIndex_, basePath = basePath_](void)
        {
            this->takeScreenshotInternal(client, cameraUrl, playerIndex, basePath);
        });
}

void QRecordingWorker::takeScreenshotInternal(
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
    const std::string& cameraUrl_,
    uint16_t playerIndex_,
    const std::string& basePath_)
{
    emit this->setCursorWaiting(true);
    bool success = false;
    std::string status;

    std::shared_ptr<rover_msgs::srv::CameraControl::Request> request
        = std::make_shared<rover_msgs::srv::CameraControl::Request>();

    request->command = rover_msgs::srv::CameraControl::Request::TAKE_PICTURE;
    request->camera_url = cameraUrl_;
    request->base_path = basePath_ + "/camera";

    if (!client_CameraControl_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "ERROR: couldn't take screenshot \ncamera control client is invalid");
        emit this->setCursorWaiting(false);
        return;
    }

    rclcpp::Client<rover_msgs::srv::CameraControl>::FutureAndRequestId future_and_request
        = client_CameraControl_->async_send_request(request);

    std::future<std::shared_ptr<rover_msgs::srv::CameraControl::Response>> future_result = std::move(future_and_request.future);

    _timer_serviceCallCamera.reset();

    bool service_call_interrupt = false;

    while (rclcpp::ok() && future_result.wait_for(std::chrono::milliseconds(SERVICE_POLL_INTERVAL)) != std::future_status::ready)
    {
        if (_timer_serviceCallCamera.isReady())
        {
            RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Service request timed out for take screenshot");
            service_call_interrupt = true;
            status = "Service call was interupted because it took too long. Probable cause: Camera is disconnected";
            break;
        }
    }

    if (future_result.valid() && !service_call_interrupt)
    {
        std::shared_ptr<rover_msgs::srv::CameraControl::Response> response = future_result.get();

        if (response != nullptr)
        {
            success = response->success;
            status = response->status;
        }
    }

    emit this->setCursorWaiting(false);
    emit this->screenshotHandledSuccessfully(success, status, playerIndex_);
}

void QRecordingWorker::startRecordingManager(
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
    const std::string& cameraUrl_,
    uint16_t playerIndex_,
    const std::string& basePath_)
{
    this->addTask(
        [this, client = client_CameraControl_, cameraUrl = cameraUrl_, playerIndex = playerIndex_, basePath = basePath_](void)
        {
            this->startRecordingInternal(client, cameraUrl, playerIndex, basePath);
        });
}

void QRecordingWorker::stopRecordingManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                                            const std::string& cameraUrl_,
                                            uint16_t playerIndex_,
                                            const std::string& basePath_)
{
    this->addTask(
        [this, client = client_CameraControl_, cameraUrl = cameraUrl_, playerIndex = playerIndex_, basePath = basePath_](void)
        {
            this->stopRecordingInternal(client, cameraUrl, playerIndex, basePath);
        });
}

void QRecordingWorker::startRecordingInternal(
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
    const std::string& cameraUrl_,
    uint16_t playerIndex_,
    const std::string& basePath_)
{
    emit this->setCursorWaiting(true);
    bool success = false;
    std::string status;

    // Explicit request type
    std::shared_ptr<rover_msgs::srv::CameraControl::Request> request
        = std::make_shared<rover_msgs::srv::CameraControl::Request>();

    request->command = rover_msgs::srv::CameraControl::Request::START_RECORDING;
    request->camera_url = cameraUrl_;
    request->base_path = basePath_ + "/camera";

    if (!client_CameraControl_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "ERROR: couldn't start recording \ncamera control client is invalid");
        emit this->setCursorWaiting(false);
        return;
    }

    // Explicit future extraction
    rclcpp::Client<rover_msgs::srv::CameraControl>::FutureAndRequestId future_and_request
        = client_CameraControl_->async_send_request(request);

    std::future<std::shared_ptr<rover_msgs::srv::CameraControl::Response>> future_result = std::move(future_and_request.future);

    _timer_serviceCallCamera.reset();

    bool service_call_interrupt = false;

    while (rclcpp::ok() && future_result.wait_for(std::chrono::milliseconds(SERVICE_POLL_INTERVAL)) != std::future_status::ready)
    {
        if (_timer_serviceCallCamera.isReady())
        {
            RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Service request timed out for start recording");
            service_call_interrupt = true;
            status = "Service call was interupted because it took too long. Probable cause: Camera is disconnected";
            break;
        }
    }

    if (future_result.valid() && !service_call_interrupt)
    {
        std::shared_ptr<rover_msgs::srv::CameraControl::Response> response = future_result.get();

        if (response != nullptr)
        {
            success = response->success;
            status = response->status;
        }
    }

    emit this->setCursorWaiting(false);
    emit this->startRecordingHandledSuccessfully(success, status, playerIndex_);
}

void QRecordingWorker::stopRecordingInternal(
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
    const std::string& cameraUrl_,
    uint16_t playerIndex_,
    const std::string& basePath_)
{
    emit this->setCursorWaiting(true);
    bool success = false;
    std::string status;

    std::shared_ptr<rover_msgs::srv::CameraControl::Request> request
        = std::make_shared<rover_msgs::srv::CameraControl::Request>();
    request->command = rover_msgs::srv::CameraControl::Request::STOP_RECORDING;
    request->camera_url = cameraUrl_;
    request->base_path = basePath_ + "/camera";

    if (!client_CameraControl_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "ERROR: camera control client is invalid");
        emit this->setCursorWaiting(false);
        return;
    }

    rclcpp::Client<rover_msgs::srv::CameraControl>::FutureAndRequestId future_and_request
        = client_CameraControl_->async_send_request(request);

    std::future<std::shared_ptr<rover_msgs::srv::CameraControl::Response>> future_result = std::move(future_and_request.future);

    _timer_serviceCallCamera.reset();

    bool service_call_interrupt = false;

    while (rclcpp::ok() && future_result.wait_for(std::chrono::milliseconds(SERVICE_POLL_INTERVAL)) != std::future_status::ready)
    {
        if (_timer_serviceCallCamera.isReady())
        {
            service_call_interrupt = true;
            RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Service request timed out for stop recording");
            status = "Service call was interupted because it took too long. Probable cause: Camera is disconnected";
            break;
        }
    }

    if (future_result.valid() && !service_call_interrupt)
    {
        std::shared_ptr<rover_msgs::srv::CameraControl::Response> response = future_result.get();
        if (response != nullptr)
        {
            success = response->success;
            status = response->status;
        }
    }

    emit this->setCursorWaiting(false);
    emit this->stopRecordingHandledSuccessfully(success, status, playerIndex_);
}
