#include "QPlayerWorker.hpp"
#include "QLogManager.hpp"
#include <QDebug>
#include <rclcpp/rclcpp.hpp>

using namespace LogUtils;

QPlayerWorker::QPlayerWorker(bool start_, QObject* parent_):
    QWorker(start_, parent_),
    _timer_serviceCall(MAX_DELAY_SERVICE_CALL)
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

void QPlayerWorker::toggleIRMode(rclcpp::Client<rover_msgs::srv::CameraIR>::SharedPtr client_cameraIR_,
                                 const std::string& ip_,
                                 uint8_t mode_)
{
    this->addTask(
        [this, client_cameraIR_, mode_, ip_](void)
        {
            this->toggleIRModeInternal(client_cameraIR_, ip_, mode_);
        });
}

void QPlayerWorker::toggleIRModeInternal(rclcpp::Client<rover_msgs::srv::CameraIR>::SharedPtr client_cameraIR_,
                                         const std::string& ip_,
                                         uint8_t mode_)
{
    if (!client_cameraIR_)
    {
        return;
    }

    rover_msgs::srv::CameraIR::Request::SharedPtr request = std::make_shared<rover_msgs::srv::CameraIR::Request>();
    request->ip = ip_;
    request->ir_mode = mode_;
    request->ir_enable = true;

    client_cameraIR_->async_send_request(request);
}