#include "QVideoRecorderWidget.hpp"

#include <Global/Helpers/QSessionFolderManager/QSessionFolderManager.hpp>
#include <Global/Helpers/QToastNotification/QToastNotification.hpp>
#include <optional>
#include <QStyle>
#include <QTimer>

QVideoRecorderWidget::QVideoRecorderWidget(const std::string& url_,
                                           uint16_t playerIndex__,
                                           std::shared_ptr<QRecordingWorker> workerThreadRecording_):
    _playerIndex(playerIndex__),
    _camURL(url_),
    _playerWorkerThreadRecording(workerThreadRecording_)
{
    connect(_playerWorkerThreadRecording.get(),
            &QRecordingWorker::screenshotHandledSuccessfully,
            this,
            &QVideoRecorderWidget::onScreenshotHandledSuccessfully);

    connect(_playerWorkerThreadRecording.get(),
            &QRecordingWorker::startRecordingHandledSuccessfully,
            this,
            &QVideoRecorderWidget::onStartRecordingHandledSuccessfully);

    connect(_playerWorkerThreadRecording.get(),
            &QRecordingWorker::stopRecordingHandledSuccessfully,
            this,
            &QVideoRecorderWidget::onStopRecordingHandledSuccessfully);

    connect(this, &QVideoRecorderWidget::updateCameraList, this, &QVideoRecorderWidget::onUpdateCameraList);

    std::optional<std::string> optionalSessionFolderPath = QSessionFolderManager::getInstance().getSessionFolderPath();
    if (optionalSessionFolderPath.has_value())
    {
        _sessionFolderPath = optionalSessionFolderPath.value();
        if (_sessionFolderPath.empty())
        {
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("No session folder found",
                                                                           "SessionFolderManager returned an empty path",
                                                                           QHelper::QToastNotification::eNotifType::ERROR);
        }
    }
    else
    {
        QHelper::QToastNotification::getInstance().notifyFromAnyThread("No session folder found",
                                                                       "SessionFolderManager couldn't return a valid path",
                                                                       QHelper::QToastNotification::eNotifType::ERROR);
    }
}

void QVideoRecorderWidget::updateCamURL(const std::string& url_)
{
    _camURL = url_;
}

void QVideoRecorderWidget::setCameraControlClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_)
{
    if (client_)
    {
        _client_cameraControlManager = client_;
        // this->autoStartRecording();
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error, couldn't access aruco detection manager client");
        if (_sButtons.screenshotButton)
        {
            _sButtons.screenshotButton->setProperty("class", "error");
            _sButtons.screenshotButton->style()->unpolish(_sButtons.screenshotButton);
            _sButtons.screenshotButton->style()->polish(_sButtons.screenshotButton);
        }
    }
}

void QVideoRecorderWidget::handleScreenshot(void) const
{
    if (_playerWorkerThreadRecording)
    {
        _playerWorkerThreadRecording->takeScreenshotManager(_client_cameraControlManager,
                                                            _camURL,
                                                            _playerIndex,
                                                            _sessionFolderPath);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"),
                     "Error, couldn't access Video Player worker. _playerWorkerThreadRecording was nullptr");
    }
}

void QVideoRecorderWidget::handleRecording(void) const
{
    if (_playerWorkerThreadRecording && _sButtons.startRecordingButton)
    {
        if (_sButtons.startRecordingButton->isChecked())
        {
            _playerWorkerThreadRecording->startRecordingManager(_client_cameraControlManager,
                                                                _camURL,
                                                                _playerIndex,
                                                                _sessionFolderPath);
        }
        else
        {
            _playerWorkerThreadRecording->stopRecordingManager(_client_cameraControlManager,
                                                               _camURL,
                                                               _playerIndex,
                                                               _sessionFolderPath);
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, couldn't access Video Player worker");
    }
}

void QVideoRecorderWidget::onScreenshotHandledSuccessfully(bool success_, const std::string& status_, uint16_t playerIndex_)
{
    if (playerIndex_ == _playerIndex && _sButtons.screenshotButton)
    {
        if (!success_)
        {
            _sButtons.screenshotButton->setProperty("class", "error");
            _sButtons.screenshotButton->style()->unpolish(_sButtons.screenshotButton);
            _sButtons.screenshotButton->style()->polish(_sButtons.screenshotButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Couldn't take screenshot",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::ERROR);
        }
        else
        {
            _sButtons.screenshotButton->setProperty("class", "success");
            _sButtons.screenshotButton->style()->unpolish(_sButtons.screenshotButton);
            _sButtons.screenshotButton->style()->polish(_sButtons.screenshotButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Screenshot taken",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::SUCCESS);
        }

        QTimer::singleShot(STYLE_RESET_TIME,
                           this,
                           [this]()
                           {
                               _sButtons.screenshotButton->setProperty("class", "normal");
                               _sButtons.screenshotButton->style()->unpolish(_sButtons.screenshotButton);
                               _sButtons.screenshotButton->style()->polish(_sButtons.screenshotButton);
                           });
    }
}

void QVideoRecorderWidget::onStartRecordingHandledSuccessfully(bool success_, const std::string& status_, uint16_t playerIndex_)
{
    if (playerIndex_ == _playerIndex && _sButtons.startRecordingButton)
    {
        if (!success_)
        {
            _sButtons.startRecordingButton->setProperty("class", "error");
            _sButtons.startRecordingButton->style()->unpolish(_sButtons.startRecordingButton);
            _sButtons.startRecordingButton->style()->polish(_sButtons.startRecordingButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Couldn't start video",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::ERROR);

            // reset after timer
            QTimer::singleShot(STYLE_RESET_TIME,
                               this,
                               [this]()
                               {
                                   _sButtons.startRecordingButton->setProperty("class", "");
                                   _sButtons.startRecordingButton->style()->unpolish(_sButtons.startRecordingButton);
                                   _sButtons.startRecordingButton->style()->polish(_sButtons.startRecordingButton);
                               });
        }
        else
        {
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Starting video recorder",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::SUCCESS);
        }
    }
}

void QVideoRecorderWidget::onStopRecordingHandledSuccessfully(bool success_, const std::string& status_, uint16_t playerIndex_)
{
    if (playerIndex_ == _playerIndex && _sButtons.startRecordingButton)
    {
        if (!success_)
        {
            _sButtons.startRecordingButton->setProperty("class", "error");
            _sButtons.startRecordingButton->style()->unpolish(_sButtons.startRecordingButton);
            _sButtons.startRecordingButton->style()->polish(_sButtons.startRecordingButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Couldn't stop recording",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::ERROR);

            // reset after timer
            QTimer::singleShot(STYLE_RESET_TIME,
                               this,
                               [this]()
                               {
                                   _sButtons.startRecordingButton->setProperty("class", "normal");
                                   _sButtons.startRecordingButton->style()->unpolish(_sButtons.startRecordingButton);
                                   _sButtons.startRecordingButton->style()->polish(_sButtons.startRecordingButton);
                               });
        }
        else
        {
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Stopping video recorder",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::SUCCESS);
        }
    }
}

void QVideoRecorderWidget::onUpdateCameraList(const std::vector<std::string>& urls)
{
    for (const std::string& url : urls)
    {
        if (url == _camURL && _sButtons.startRecordingButton)
        {
            if (!_sButtons.startRecordingButton->isChecked())
            {
                _sButtons.startRecordingButton->setChecked(true);
                _sButtons.startRecordingButton->style()->unpolish(_sButtons.startRecordingButton);
                _sButtons.startRecordingButton->style()->polish(_sButtons.startRecordingButton);
            }
            return;
        }
    }

    // if cam_url wasn't found in vector and we're currently recording
    if (_sButtons.startRecordingButton && _sButtons.startRecordingButton->isChecked())
    {
        std::string error_message = "Recording on " + _camURL + " was stopped unexpectedly";
        _sButtons.startRecordingButton->setProperty("class", "normal");
        _sButtons.startRecordingButton->setChecked(false);
        _sButtons.startRecordingButton->style()->unpolish(_sButtons.startRecordingButton);
        _sButtons.startRecordingButton->style()->polish(_sButtons.startRecordingButton);
        QHelper::QToastNotification::getInstance().notifyFromAnyThread("Recording stopped",
                                                                       error_message,
                                                                       QHelper::QToastNotification::eNotifType::WARNING);
    }
}

void QVideoRecorderWidget::CB_srvAvailable(bool available_)
{
    if (_sButtons.screenshotButton && _sButtons.startRecordingButton)
    {
        if (!available_)
        {
            _sButtons.screenshotButton->setEnabled(false);
            _sButtons.startRecordingButton->setEnabled(false);
        }
        else if (!_sButtons.screenshotButton->isEnabled() || !_sButtons.startRecordingButton->isEnabled())
        {
            _sButtons.screenshotButton->setEnabled(true);
            _sButtons.startRecordingButton->setEnabled(true);
        }
    }
}

void QVideoRecorderWidget::setButtons(const sRecordingButtons& buttons_)
{
    _sButtons = buttons_;

    if (_sButtons.screenshotButton && _sButtons.startRecordingButton)
    {
        connect(_sButtons.screenshotButton, &QPushButton::clicked, this, &QVideoRecorderWidget::handleScreenshot);
        connect(_sButtons.startRecordingButton, &QPushButton::clicked, this, &QVideoRecorderWidget::handleRecording);
    }
}

void QVideoRecorderWidget::emitUpdateCameraList(const std::vector<std::string>& urls)
{
    emit this->updateCameraList(urls);
}

void QVideoRecorderWidget::autoStartRecording(void)
{
    if (_sButtons.startRecordingButton)
    {
        _sButtons.startRecordingButton->setChecked(true);
        _playerWorkerThreadRecording->startRecordingManager(_client_cameraControlManager,
                                                            _camURL,
                                                            _playerIndex,
                                                            _sessionFolderPath);
    }
}