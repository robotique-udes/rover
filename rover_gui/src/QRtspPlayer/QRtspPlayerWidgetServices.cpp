#include "QRtspPlayer/QRtspPlayerWidgetHeader.hpp"
#include "QLoggingMacros.hpp"
#include <QMessageBox>

using CameraControlServiceType = rover_msgs::srv::CameraControl;
using ArucoDetectionServiceType = rover_msgs::srv::ArucoDetection;
using ArucoMessageType = rover_msgs::msg::Aruco;

void RtspPlayerWidget::initializeRosServices(std::shared_ptr<rclcpp::Node> node_)
{
    if (!node_) {
        LOG_ERROR_TARGET("RtspPlayer", "Cannot initialize ROS services: Node is null", this->_widgetId.toUtf8().constData());
        return;
    }

    _rosNode = node_;

    try {
        
        _cameraControlClient = _rosNode->create_client<CameraControlServiceType>("/rover/video/media_server");
        _arucoDetectionClient = _rosNode->create_client<ArucoDetectionServiceType>("/rover/auxiliary/aruco/manager");
        
        _isCameraServiceAvailable = _cameraControlClient->wait_for_service(std::chrono::milliseconds(500));
        _isArucoServiceAvailable = _arucoDetectionClient->wait_for_service(std::chrono::milliseconds(500));
        
        if (_isCameraServiceAvailable) {
            LOG_INFO_TARGET("RtspPlayer", "Camera control service connected", this->_widgetId.toUtf8().constData());
        } else {
            LOG_WARNING_TARGET("RtspPlayer", "Camera control service not available", this->_widgetId.toUtf8().constData());
            
            startServiceAvailabilityPolling();
        }
        
        if (_isArucoServiceAvailable) {
            LOG_INFO_TARGET("RtspPlayer", "Aruco detection service connected", this->_widgetId.toUtf8().constData());
        } else {
            LOG_WARNING_TARGET("RtspPlayer", "Aruco detection service not available", this->_widgetId.toUtf8().constData());
            
            startServiceAvailabilityPolling();
        }
        
       
        _arucoSubscription = _rosNode->create_subscription<ArucoMessageType>(
            "/rover/video/aruco", 10,
            std::bind(&RtspPlayerWidget::handleArucoDetection, this, std::placeholders::_1));
            
        LOG_INFO_TARGET("RtspPlayer", "ROS services initialized", this->_widgetId.toUtf8().constData());
    }
    catch (const std::exception& e) {
        LOG_ERROR_TARGET("RtspPlayer", "Error initializing ROS services: " + QString(e.what()), this->_widgetId.toUtf8().constData());
        
        startServiceAvailabilityPolling();
    }
}

void RtspPlayerWidget::startServiceAvailabilityPolling()
{
    
    if (!_servicePollingActive) {
        _servicePollingActive = true;
        
        connect(&_servicePollingTimer, &QTimer::timeout, this, &RtspPlayerWidget::checkServiceAvailability);
        
        _servicePollingTimer.setInterval(5000);
        _servicePollingTimer.start();
        
        LOG_INFO_TARGET("RtspPlayer", "Started service availability polling", this->_widgetId.toUtf8().constData());
    }
}

void RtspPlayerWidget::checkServiceAvailability()
{
    if (!_isCameraServiceAvailable && _cameraControlClient) {
        bool available = _cameraControlClient->wait_for_service(std::chrono::milliseconds(200));
        if (available && !_isCameraServiceAvailable) {
            _isCameraServiceAvailable = true;
            LOG_INFO_TARGET("RtspPlayer", "Camera control service is now available", this->_widgetId.toUtf8().constData());

            if (_state == PlayerState::Streaming) {
                this->_screenshotButton->setEnabled(true);
                this->_recordButton->setEnabled(true);
            }
        }
    }
    
    if (!_isArucoServiceAvailable && _arucoDetectionClient) {
        bool available = _arucoDetectionClient->wait_for_service(std::chrono::milliseconds(200));
        if (available && !_isArucoServiceAvailable) {
            _isArucoServiceAvailable = true;
            LOG_INFO_TARGET("RtspPlayer", "Aruco detection service is now available", this->_widgetId.toUtf8().constData());
      
            if (_state == PlayerState::Streaming) {
                this->_arucoButton->setEnabled(true);
            }
        }
    }

    if (_isCameraServiceAvailable && _isArucoServiceAvailable) {
        _servicePollingTimer.stop();
        _servicePollingActive = false;
        LOG_INFO_TARGET("RtspPlayer", "All services are available, stopping polling", this->_widgetId.toUtf8().constData());
    }
}

void RtspPlayerWidget::handleArucoDetection(const ArucoMessageType::SharedPtr msg)
{
    
    if (msg->cam_url != this->_lastStreamUrl.toStdString()) {
        return;
    }
    
    if (msg->valid && !msg->id.empty()) {
        QString idText = "Ids: ";
        size_t count = 0;
        
        for (const auto& id : msg->id) {
            if (count < NBR_IDS_TO_DISPLAY) {
                idText += QString::number(id) + " ";
                count++;
            } else {
                idText += "...";
                break;
            }
        }
        
        QMetaObject::invokeMethod(this, [this, idText]() {
            this->_arucoIdsTextBox->setText(idText);
        }, Qt::QueuedConnection);
        
        if (!this->_arucoButton->isChecked()) {
            LOG_DEBUG_TARGET("RtspPlayer", "Received Aruco detection but detection is not enabled", this->_widgetId.toUtf8().constData());
        }
    } else if (!msg->valid && this->_arucoButton->isChecked()) {

        QMetaObject::invokeMethod(this, [this]() {
            this->_arucoIdsTextBox->setText("Ids: None");
        }, Qt::QueuedConnection);
    }
}

void RtspPlayerWidget::onArucoButtonToggled(bool checked)
{
    if (!_isArucoServiceAvailable) {
        QMessageBox::warning(this, "Service Unavailable", 
                           "Aruco detection service is not available.\n\n"
                           "Please make sure the ROS node is running.");
        this->_arucoButton->setChecked(!checked);
        return;
    }
    
    try {
        auto request = std::make_shared<ArucoDetectionServiceType::Request>();
        request->camera_url = _lastStreamUrl.toStdString();
        
        if (checked) {
            request->command = ArucoDetectionServiceType::Request::START;
            this->_arucoButton->setStyleSheet("background-color: #5cb85c; color: white;");
            LOG_INFO_TARGET("RtspPlayer", "Starting Aruco detection", this->_widgetId.toUtf8().constData());
        } else {
            request->command = ArucoDetectionServiceType::Request::STOP;
            this->_arucoButton->setStyleSheet("");
            this->_arucoIdsTextBox->setText("Ids: ");
            LOG_INFO_TARGET("RtspPlayer", "Stopping Aruco detection", this->_widgetId.toUtf8().constData());
        }
        
        auto callback = [this, checked](rclcpp::Client<ArucoDetectionServiceType>::SharedFuture future) {
            try {
                auto response = future.get();
                if (!response->success) {
                    LOG_ERROR_TARGET("RtspPlayer", "Aruco detection service call failed", this->_widgetId.toUtf8().constData());
                    
                    QMetaObject::invokeMethod(this, [this, checked]() {
                        this->_arucoButton->blockSignals(true);
                        this->_arucoButton->setChecked(!checked);
                        this->_arucoButton->blockSignals(false);
                        
                        if (!checked) {
                            this->_arucoButton->setStyleSheet("background-color: #5cb85c; color: white;");
                        } else {
                            this->_arucoButton->setStyleSheet("");
                        }
                        
                        QMessageBox::warning(this, "Service Error", 
                                          "Failed to " + QString(checked ? "start" : "stop") + " Aruco detection.");
                    }, Qt::QueuedConnection);
                } else {
                    LOG_INFO_TARGET("RtspPlayer", "Aruco detection service call succeeded", this->_widgetId.toUtf8().constData());
                }
            }
            catch (const std::exception& e) {
                LOG_ERROR_TARGET("RtspPlayer", "Exception in Aruco service callback: " + QString(e.what()), this->_widgetId.toUtf8().constData());
            }
        };
        
        _arucoDetectionClient->async_send_request(request, callback);
    }
    catch (const std::exception& e) {
        LOG_ERROR_TARGET("RtspPlayer", "Exception sending Aruco request: " + QString(e.what()), this->_widgetId.toUtf8().constData());
        
        this->_arucoButton->blockSignals(true);
        this->_arucoButton->setChecked(!checked);
        this->_arucoButton->blockSignals(false);
        
        QMessageBox::warning(this, "Service Error", 
                           "Error sending request: " + QString(e.what()));
    }
}

void RtspPlayerWidget::handleScreenshotRequest()
{
    if (!_isCameraServiceAvailable) {
        QMessageBox::warning(this, "Service Unavailable", 
                           "Media Server is not available.\n\n"
                           "Please make sure the ROS node is running.");
        return;
    }
    
    try {
        LOG_INFO_TARGET("RtspPlayer", "Taking screenshot", this->_widgetId.toUtf8().constData());
        
        auto request = std::make_shared<CameraControlServiceType::Request>();
        request->camera_url = _lastStreamUrl.toStdString();
        request->command = CameraControlServiceType::Request::TAKE_PICTURE;
      
        request->capture_name = this->_widgetId.toStdString();
        
        auto callback = [this](rclcpp::Client<CameraControlServiceType>::SharedFuture future) {
            try {
                auto response = future.get();
                if (response->success) {
                    LOG_INFO_TARGET("RtspPlayer", "Screenshot taken successfully: " + QString::fromStdString(response->status), this->_widgetId.toUtf8().constData());
                    
                    QMetaObject::invokeMethod(this, [this, status = response->status]() {
                        QMessageBox::information(this, "Screenshot Taken", 
                                              QString::fromStdString(status));
                    }, Qt::QueuedConnection);
                } else {
                    LOG_ERROR_TARGET("RtspPlayer", "Failed to take screenshot: " + QString::fromStdString(response->status), this->_widgetId.toUtf8().constData());
                    
                    QMetaObject::invokeMethod(this, [this, status = response->status]() {
                        QMessageBox::warning(this, "Screenshot Failed", 
                                          QString::fromStdString(status));
                    }, Qt::QueuedConnection);
                }
            }
            catch (const std::exception& e) {
                LOG_ERROR_TARGET("RtspPlayer", "Exception in screenshot callback: " + QString(e.what()), this->_widgetId.toUtf8().constData());
            }
        };
        
        _cameraControlClient->async_send_request(request, callback);
    }
    catch (const std::exception& e) {
        LOG_ERROR_TARGET("RtspPlayer", "Exception sending screenshot request: " + QString(e.what()), this->_widgetId.toUtf8().constData());
        QMessageBox::warning(this, "Service Error", 
                           "Error sending request: " + QString(e.what()));
    }
}

void RtspPlayerWidget::handleRecordingRequest(bool checked)
{
    if (!_isCameraServiceAvailable) {
        QMessageBox::warning(this, "Service Unavailable", 
                           "Media Server is not available.\n\n"
                           "Please make sure the ROS node is running.");
        
        this->_recordButton->setChecked(!checked);
        return;
    }

    if (_state != PlayerState::Streaming) {
        QMessageBox::warning(this, "Stream Error", 
                           "Cannot " + QString(checked ? "start" : "stop") + " recording without an active stream.");
    
        this->_recordButton->setChecked(false);
        return;
    }
    
    try {
        auto request = std::make_shared<CameraControlServiceType::Request>();
        request->camera_url = _lastStreamUrl.toStdString();
        
        if (checked) {

            request->command = CameraControlServiceType::Request::START_RECORDING;
            request->capture_name = this->_widgetId.toStdString();
            LOG_INFO_TARGET("RtspPlayer", "Starting recording", this->_widgetId.toUtf8().constData());
            
            _recordButton->setIcon(QIcon(":/icons/record_on.png"));
        } else {

            request->command = CameraControlServiceType::Request::STOP_RECORDING;
            LOG_INFO_TARGET("RtspPlayer", "Stopping recording", this->_widgetId.toUtf8().constData());
            
            _recordButton->setIcon(QIcon(":/icons/record_off.png"));
        }
        
        auto callback = [this, checked](rclcpp::Client<CameraControlServiceType>::SharedFuture future) {
            try {
                auto response = future.get();
                if (!response->success) {
                    LOG_ERROR_TARGET("RtspPlayer", "Recording service call failed: " + QString::fromStdString(response->status), this->_widgetId.toUtf8().constData());
                    
                    QMetaObject::invokeMethod(this, [this, checked]() {
                        this->_recordButton->blockSignals(true);
                        this->_recordButton->setChecked(!checked);
                        this->_recordButton->blockSignals(false);
                        
                        if (!checked) {
                            this->_recordButton->setIcon(QIcon(":/icons/record_on.png"));
                        } else {
                            this->_recordButton->setIcon(QIcon(":/icons/record_off.png"));
                        }
                        
                        QMessageBox::warning(this, "Service Error", 
                                          "Failed to " + QString(checked ? "start" : "stop") + " recording.");
                    }, Qt::QueuedConnection);
                } else {
                    LOG_INFO_TARGET("RtspPlayer", "Recording service call succeeded: " + QString::fromStdString(response->status), this->_widgetId.toUtf8().constData());
                    
                    QMetaObject::invokeMethod(this, [this, checked, status = response->status]() {
                        QMessageBox::information(this, checked ? "Recording Started" : "Recording Stopped", 
                                              QString::fromStdString(status));
                    }, Qt::QueuedConnection);
                }
            }
            catch (const std::exception& e) {
                LOG_ERROR_TARGET("RtspPlayer", "Exception in recording callback: " + QString(e.what()), this->_widgetId.toUtf8().constData());
            }
        };
        
        _cameraControlClient->async_send_request(request, callback);
    }
    catch (const std::exception& e) {
        LOG_ERROR_TARGET("RtspPlayer", "Exception sending recording request: " + QString(e.what()), this->_widgetId.toUtf8().constData());
        
        this->_recordButton->blockSignals(true);
        this->_recordButton->setChecked(!checked);
        this->_recordButton->blockSignals(false);
        
        QMessageBox::warning(this, "Service Error", 
                           "Error sending request: " + QString(e.what()));
    }
}