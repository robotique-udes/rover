#include "QPlayerWorker.hpp"
#include "QLogManager.hpp"
#include <QDebug>
#include <rclcpp/rclcpp.hpp>

using namespace LogUtils;

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
    std::string _camURL_,
    uint16_t playerIndex_,
    bool start_)
{
    bool success = false;

    std::shared_ptr<rover_msgs::srv::ArucoDetection::Request> request
        = std::make_shared<rover_msgs::srv::ArucoDetection::Request>();

    if (start_)
    {
        request->command = rover_msgs::srv::ArucoDetection::Request::START;
    }
    else
    {
        request->command = rover_msgs::srv::ArucoDetection::Request::STOP;
    }

    request->camera_url = _camURL_;

    UI_LOG_INFO(ARUCO_DETECTION,
                QString("Sending aruco detection %1 request for camera %2 (tag: %3)")
                    .arg(start_ ? "START" : "STOP")
                    .arg(QString::fromStdString(_camURL_))
                    .arg(playerIndex_),
                nullptr);

    rclcpp::Client<rover_msgs::srv::ArucoDetection>::FutureAndRequestId future_and_request
        = client_ArucoDetectionManager_->async_send_request(request);

    std::future<std::shared_ptr<rover_msgs::srv::ArucoDetection::Response>> future_result = std::move(future_and_request.future);

    _timer_serviceCall.reset();
    bool service_call_interrupted = false;

    while (rclcpp::ok() && future_result.wait_for(std::chrono::milliseconds(SERVICE_POLL_INTERVAL)) != std::future_status::ready)
    {
        if (_timer_serviceCall.isReady())
        {
            service_call_interrupted = true;
            break;
        }
    }

    if (future_result.valid() && !service_call_interrupted)
    {
        std::shared_ptr<rover_msgs::srv::ArucoDetection::Response> response = future_result.get();

        if (response != nullptr)
        {
            success = response->success;
        }
    }

    if (success)
    {
        UI_LOG_INFO(ARUCO_DETECTION,
                    QString("Aruco detection request successful for camera %1 (tag: %2)")
                        .arg(QString::fromStdString(_camURL_))
                        .arg(playerIndex_),
                    nullptr);
    }
    else
    {
        UI_LOG_ERROR(ARUCO_DETECTION,
                     QString("Aruco detection request failed for camera %1 (tag: %2)")
                         .arg(QString::fromStdString(_camURL_))
                         .arg(playerIndex_),
                     nullptr);
    }

    emit this->detectionHandledSuccessfully(success, playerIndex_);
}

void QPlayerWorker::manageDetection(
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager_,
    std::string _camURL_,
    uint16_t playerIndex_,
    bool start_)
{
    this->addTask(
        [this, client_ArucoDetectionManager_, _camURL_, playerIndex_, start_](void)
        {
            this->manageDetectionInternal(client_ArucoDetectionManager_, _camURL_, playerIndex_, start_);
        });
}

void QPlayerWorker::updateDetectionManager(
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager_)
{
    this->addTask(
        [this, client_ArucoDetectionManager_](void)
        {
            this->updateDetectionInternal(client_ArucoDetectionManager_);
        });
}

void QPlayerWorker::updateDetectionInternal(
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager_)
{
    bool success = false;

    std::shared_ptr<rover_msgs::srv::ArucoDetection::Request> request
        = std::make_shared<rover_msgs::srv::ArucoDetection::Request>();

    request->command = rover_msgs::srv::ArucoDetection::Request::INFO;
    request->camera_url = "";

    if (!client_ArucoDetectionManager_)
    {
        return;
    }

    UI_LOG_DEBUG(ARUCO_DETECTION, "Sending aruco detection INFO request", nullptr);

    auto result = client_ArucoDetectionManager_->async_send_request(request);

    _timer_serviceCall.reset();
    bool service_call_interrupt = false;

    while (rclcpp::ok() && result.wait_for(std::chrono::milliseconds(SERVICE_POLL_INTERVAL)) != std::future_status::ready)
    {
        if (_timer_serviceCall.isReady())
        {
            service_call_interrupt = true;
            break;
        }
    }

    std::vector<std::string> liveURLs;

    if (result.valid() && !service_call_interrupt)
    {
        std::shared_ptr<rover_msgs::srv::ArucoDetection::Response> response = result.get();

        if (response != nullptr)
        {
            liveURLs = response->urls;
            success = true;
        }
    }

    emit this->arucoServerInfoFailed(success);
    emit this->urlFoundInDetection(liveURLs);
}

void QPlayerWorker::takeScreenshotManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                                          std::string cameraUrl_,
                                          uint16_t playerIndex_, std::string basePath_)
{
    this->addTask(
        [this, client_CameraControl_, cameraUrl_, playerIndex_, basePath_](void)
        {
            this->takeScreenshotInternal(client_CameraControl_, cameraUrl_, playerIndex_, basePath_);
        });
}

void QPlayerWorker::takeScreenshotInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                                           std::string cameraUrl_,
                                           uint16_t playerIndex_, std::string basePath_)
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

void QPlayerWorker::startRecordingManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                                          std::string cameraUrl_,
                                          uint16_t playerIndex_, std::string basePath_)
{
    this->addTask(
        [this, client_CameraControl_, cameraUrl_, playerIndex_, basePath_](void)
        {
            this->startRecordingInternal(client_CameraControl_, cameraUrl_, playerIndex_, basePath_);
        });
}

void QPlayerWorker::stopRecordingManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                                         std::string cameraUrl_,
                                         uint16_t playerIndex_, std::string basePath_)
{
    this->addTask(
        [this, client_CameraControl_, cameraUrl_, playerIndex_, basePath_](void)
        {
            this->stopRecordingInternal(client_CameraControl_, cameraUrl_, playerIndex_, basePath_);
        });
}

void QPlayerWorker::startRecordingInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                                           std::string cameraUrl_,
                                           uint16_t playerIndex_, std::string basePath_)
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

void QPlayerWorker::stopRecordingInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                                          std::string cameraUrl_,
                                          uint16_t playerIndex_, std::string basePath_)
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
