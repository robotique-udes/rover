#include "QPlayerWorker.hpp"

#include <QDebug>
#include <rclcpp/rclcpp.hpp>

QPlayerWorker::QPlayerWorker(bool start_, QObject* parent_):
    QWorker(start_, parent_),
    _timer_serviceCall(MAX_DELAY_SERVICE_CALL),
    _timer_serviceCallCamera(MAX_DELAY_SERVICE_CALL)
{
}

QPlayerWorker::~QPlayerWorker()
{
    this->finish();
}

void QPlayerWorker::manageDetectionInternal(
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager_,
    std::string _camURL,
    uint16_t tag_,
    bool start_)
{
    bool success = false;
    auto request = std::make_shared<rover_msgs::srv::ArucoDetection::Request>();

    if (start_)
    {
        request->command = rover_msgs::srv::ArucoDetection::Request::START;
    }
    else
    {
        request->command = rover_msgs::srv::ArucoDetection::Request::STOP;
    }

    request->camera_url = _camURL;

    auto result = client_ArucoDetectionManager_->async_send_request(request);

    _timer_serviceCall.reset();
    bool service_call_interrupted = false;

    while (rclcpp::ok() && result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
    {
        if (_timer_serviceCall.isReady())
        {
            service_call_interrupted = true;
            break;
        }
    }

    if (result.valid() && !service_call_interrupted)
    {
        std::shared_ptr<rover_msgs::srv::ArucoDetection::Response> response = result.get();

        if (response != nullptr)
        {
            success = response->success;
        }
    }
    emit detectionHandledSuccessfully(success, tag_);
}

void QPlayerWorker::manageDetection(
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager_,
    std::string _camURL,
    uint16_t tag_,
    bool start_)
{
    this->addTask(
        [this, client_ArucoDetectionManager_, _camURL, tag_, start_](void)
        {
            this->manageDetectionInternal(client_ArucoDetectionManager_, _camURL, tag_, start_);
        });
}

void QPlayerWorker::updateDetectionManager(
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager)
{
    this->addTask(
        [this, client_ArucoDetectionManager](void)
        {
            this->updateDetectionInternal(client_ArucoDetectionManager);
        });
}

void QPlayerWorker::updateDetectionInternal(
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager_)
{
    bool success = false;

    auto request = std::make_shared<rover_msgs::srv::ArucoDetection::Request>();

    request->command = rover_msgs::srv::ArucoDetection::Request::INFO;

    request->camera_url = "";

    if (!client_ArucoDetectionManager_)
    {
        return;
    }

    auto result = client_ArucoDetectionManager_->async_send_request(request);

    _timer_serviceCall.reset();

    bool service_call_interrupt = false;

    while (rclcpp::ok() && result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
    {
        if (_timer_serviceCall.isReady())
        {
            service_call_interrupt = true;
            break;
        }
    }

    std::vector<std::string> liveURLs;

    if (result.valid())
    {
        if (!service_call_interrupt)
        {
            std::shared_ptr<rover_msgs::srv::ArucoDetection::Response> response = result.get();

            if (response != nullptr)
            {
                liveURLs = response->urls;
                success = true;
            }
        }
    }

    emit arucoServerInfoFailed(success);
    emit urlFoundInDetection(liveURLs);
}

void QPlayerWorker::takeScreenshotManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                                          std::string camera_URL_,
                                          uint16_t tag_)
{
    this->addTask(
        [this, client_CameraControl_, camera_URL_, tag_](void)
        {
            this->takeScreenshotInternal(client_CameraControl_, camera_URL_, tag_);
        });
}

void QPlayerWorker::takeScreenshotInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                                           std::string camera_URL_,
                                           uint16_t tag_)
{
    emit setCursorWaiting(true);
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
            RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Service request timed out for take screenshot");
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

    emit setCursorWaiting(false);
    emit screenshotHandledSuccessfully(success, status, tag_);
}

void QPlayerWorker::startRecordingManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                                          std::string camera_URL_,
                                          uint16_t tag_)
{
    this->addTask(
        [this, client_CameraControl_, camera_URL_, tag_](void)
        {
            this->startRecordingInternal(client_CameraControl_, camera_URL_, tag_);
        });
}

void QPlayerWorker::stopRecordingManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                                         std::string camera_URL_,
                                         uint16_t tag_)
{
    this->addTask(
        [this, client_CameraControl_, camera_URL_, tag_](void)
        {
            this->stopRecordingInternal(client_CameraControl_, camera_URL_, tag_);
        });
}

void QPlayerWorker::startRecordingInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                                           std::string camera_URL_,
                                           uint16_t tag_)
{
    emit setCursorWaiting(true);
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
            RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Service request timed out for start recording");
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

    emit setCursorWaiting(false);
    emit startRecordingHandledSuccessfully(success, status, tag_);
}

void QPlayerWorker::stopRecordingInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                                          std::string camera_URL_,
                                          uint16_t tag_)
{
    emit setCursorWaiting(true);
    bool success = false;
    std::string status;

    auto request = std::make_shared<rover_msgs::srv::CameraControl::Request>();

    request->command = rover_msgs::srv::CameraControl::Request::STOP_RECORDING;

    request->camera_url = camera_URL_;

    if (!client_CameraControl_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Service request timed out");
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
            RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Service request timed out for stop recording");
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

    emit setCursorWaiting(false);
    emit stopRecordingHandledSuccessfully(success, status, tag_);
    return;
}