#include "QVideoRecorderWidget.hpp"

#include <Global/Helpers/QSessionFolderManager/QSessionFolderManager.hpp>
#include <Global/Helpers/QToastNotification/QToastNotification.hpp>
#include <optional>
#include <QStyle>
#include <QTimer>

QVideoRecorderWidget::QVideoRecorderWidget(sRecordingButtons buttons_,
                                           std::string url_,
                                           uint16_t playerIndex__,
                                           std::shared_ptr<QRecordingWorker> workerThreadRecording_):
    _playerIndex(playerIndex__),
    _camURL(url_),
    _startRecordingButton(buttons_.startRecordingButton),
    _screenshotButton(buttons_.screenshotButton),
    _playerWorkerThreadRecording(workerThreadRecording_)
{
    connect(_screenshotButton, &QPushButton::clicked, this, &QVideoRecorderWidget::handleScreenshot);
    connect(_playerWorkerThreadRecording.get(),
            &QRecordingWorker::screenshotHandledSuccessfully,
            this,
            &QVideoRecorderWidget::onScreenshotHandledSuccessfully);

    connect(_startRecordingButton, &QPushButton::clicked, this, &QVideoRecorderWidget::handleRecording);
    connect(_playerWorkerThreadRecording.get(),
            &QRecordingWorker::startRecordingHandledSuccessfully,
            this,
            &QVideoRecorderWidget::onStartRecordingHandledSuccessfully);

    connect(_playerWorkerThreadRecording.get(),
            &QRecordingWorker::stopRecordingHandledSuccessfully,
            this,
            &QVideoRecorderWidget::onStopRecordingHandledSuccessfully);

    std::optional<std::string> optionalSessionFolderPath = QSessionFolderManager::getInstance().getSessionFolderPath();
    if (optionalSessionFolderPath.has_value())
    {
        _sessionFolderPath = *optionalSessionFolderPath;
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

QVideoRecorderWidget::~QVideoRecorderWidget() {}

void QVideoRecorderWidget::updateCamURL(std::string& url_)
{
    _camURL = url_;
}

void QVideoRecorderWidget::setCameraControlClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_)
{
    if (client_)
    {
        _client_cameraControlManager = client_;
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error, couldn't access aruco detection manager client");
        _screenshotButton->setProperty("class", "error");
        _screenshotButton->style()->unpolish(_screenshotButton);
        _screenshotButton->style()->polish(_screenshotButton);
    }
}

void QVideoRecorderWidget::handleScreenshot(void)
{
    if (_playerWorkerThreadRecording.get() != nullptr)
    {
        _playerWorkerThreadRecording->takeScreenshotManager(_client_cameraControlManager,
                                                            _camURL,
                                                            _playerIndex,
                                                            _sessionFolderPath);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, couldn't access Video Player worker");
    }
    return;
}

void QVideoRecorderWidget::handleRecording(void)
{
    if (_playerWorkerThreadRecording.get() != nullptr)
    {
        if (_startRecordingButton->isChecked())
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
    return;
}

void QVideoRecorderWidget::onScreenshotHandledSuccessfully(bool success_, std::string status_, uint16_t playerIndex_)
{
    if (playerIndex_ == _playerIndex)
    {
        if (!success_)
        {
            _screenshotButton->setProperty("class", "error");
            _screenshotButton->style()->unpolish(_screenshotButton);
            _screenshotButton->style()->polish(_screenshotButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Couldn't take screenshot",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::ERROR);
        }
        else
        {
            _screenshotButton->setProperty("class", "success");
            _screenshotButton->style()->unpolish(_screenshotButton);
            _screenshotButton->style()->polish(_screenshotButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Screenshot taken",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::SUCCESS);
        }

        QTimer::singleShot(STYLE_RESET_TIME,
                           this,
                           [this]()
                           {
                               _screenshotButton->setProperty("class", "normal");
                               _screenshotButton->style()->unpolish(_screenshotButton);
                               _screenshotButton->style()->polish(_screenshotButton);
                           });
    }
    return;
}

void QVideoRecorderWidget::onStartRecordingHandledSuccessfully(bool success_, std::string status_, uint16_t playerIndex_)
{
    if (playerIndex_ == _playerIndex)
    {
        if (!success_)
        {
            _startRecordingButton->setProperty("class", "error");
            _startRecordingButton->style()->unpolish(_startRecordingButton);
            _startRecordingButton->style()->polish(_startRecordingButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Couldn't start video",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::ERROR);

            // reset after timer
            QTimer::singleShot(STYLE_RESET_TIME,
                               this,
                               [this]()
                               {
                                   _startRecordingButton->setProperty("class", "");
                                   _startRecordingButton->style()->unpolish(_startRecordingButton);
                                   _startRecordingButton->style()->polish(_startRecordingButton);
                               });
        }
        else
        {
            _startRecordingButton->setProperty("class", "success");
            _startRecordingButton->setIcon(QIcon::fromTheme("media-playback-stop"));
            _startRecordingButton->style()->unpolish(_startRecordingButton);
            _startRecordingButton->style()->polish(_startRecordingButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Video started",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::SUCCESS);
        }
    }
    return;
}

void QVideoRecorderWidget::onStopRecordingHandledSuccessfully(bool success_, std::string status_, uint16_t playerIndex_)
{
    if (playerIndex_ == _playerIndex)
    {
        if (!success_)
        {
            _startRecordingButton->setProperty("class", "error");
            _startRecordingButton->style()->unpolish(_startRecordingButton);
            _startRecordingButton->style()->polish(_startRecordingButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Couldn't stop recording",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::ERROR);

            // reset after timer
            QTimer::singleShot(STYLE_RESET_TIME,
                               this,
                               [this]()
                               {
                                   _startRecordingButton->setProperty("class", "normal");
                                   _startRecordingButton->style()->unpolish(_startRecordingButton);
                                   _startRecordingButton->style()->polish(_startRecordingButton);
                               });
        }
        else
        {
            _startRecordingButton->setProperty("class", "normal");
            _startRecordingButton->setIcon(QIcon::fromTheme("media-record"));
            _startRecordingButton->style()->unpolish(_startRecordingButton);
            _startRecordingButton->style()->polish(_startRecordingButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Recording stopped",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::SUCCESS);
        }
    }
    return;
}

void QVideoRecorderWidget::CB_cameraListUpdate(std::vector<std::string> urls)
{
    for (const auto& url : urls)
    {
        if (url == _camURL)
        {
            if (!_startRecordingButton->isChecked())
            {
                _startRecordingButton->setChecked(true);
                _startRecordingButton->setProperty("class", "success");
                _startRecordingButton->setIcon(QIcon::fromTheme("media-playback-stop"));
                _startRecordingButton->style()->unpolish(_startRecordingButton);
                _startRecordingButton->style()->polish(_startRecordingButton);
            }
            return;
        }
    }

    // if cam_url wasn't found in vector and we're currently recording
    if (_startRecordingButton->isChecked())
    {
        std::string error_message = "Recording on " + _camURL + " was stopped unexpectedly";
        _startRecordingButton->setChecked(false);
        _startRecordingButton->setIcon(QIcon::fromTheme("media-record"));
        _startRecordingButton->style()->unpolish(_startRecordingButton);
        _startRecordingButton->style()->polish(_startRecordingButton);
        QHelper::QToastNotification::getInstance().notifyFromAnyThread("Recording stopped",
                                                                       error_message,
                                                                       QHelper::QToastNotification::eNotifType::WARNING);
    }
}

void QVideoRecorderWidget::CB_serviceCameraControlAvailable(bool available_)
{
    if (!available_)
    {
        _screenshotButton->setEnabled(false);
        _startRecordingButton->setEnabled(false);
    }
    else if (!_screenshotButton->isEnabled() || !_startRecordingButton->isEnabled())
    {
        _screenshotButton->setEnabled(true);
        _startRecordingButton->setEnabled(true);
    }
}
