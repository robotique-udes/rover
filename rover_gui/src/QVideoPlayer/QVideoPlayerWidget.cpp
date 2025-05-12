#include "QVideoPlayerWidget.hpp"
#include <QStyle>

QVideoPlayerWidget::QVideoPlayerWidget(std::shared_ptr<rclcpp::Node> guiNode_,
                                       std::string url_,
                                       uint16_t tag_,
                                       std::shared_ptr<QPlayerWorker> worker_):
    _node(guiNode_),
    _camURL(url_),
    _tag(tag_),
    _playerWorkerThread(worker_)
{
    _defaultCamUrl = _camURL;
    _ui.setupUi(this);

    connect(_ui.arucoPushButton, &QPushButton::clicked, this, &QVideoPlayerWidget::handleArucoDetection);
    connect(_playerWorkerThread.get(),
            &QPlayerWorker::detectionHandledSuccessfully,
            this,
            &QVideoPlayerWidget::onDetectionHandledSuccessfully);
    connect(_ui.playPauseButton, &QPushButton::clicked, this, &QVideoPlayerWidget::handlePlayPauseButton);
    connect(_playerWorkerThread.get(), &QPlayerWorker::arucoServerInfoFailed, this, &QVideoPlayerWidget::onArucoServerInfoFailed);

    connect(_ui.rtspTextBox, &QLineEdit::textChanged, this, &QVideoPlayerWidget::updateCamURL);
    connect(_ui.defaultStreamPushButton, &QPushButton::clicked, this, &QVideoPlayerWidget::setURLToDefault);
    connect(this, &QVideoPlayerWidget::arucoCameraFailure, this, &QVideoPlayerWidget::onArucoCameraFailed);

    connect(_ui.ScreenshotButton, &QPushButton::clicked, this, &QVideoPlayerWidget::handleScreenshot);
    connect(_playerWorkerThread.get(),
            &QPlayerWorker::screenshotHandledSuccessfully,
            this,
            &QVideoPlayerWidget::onScreenshotHandledSuccessfully);

    connect(_ui.startRecordingButton, &QPushButton::clicked, this, &QVideoPlayerWidget::handleRecording);

    connect(_playerWorkerThread.get(),
            &QPlayerWorker::startRecordingHandledSuccessfully,
            this,
            &QVideoPlayerWidget::onStartRecordingHandledSuccessfully);

    connect(_playerWorkerThread.get(),
            &QPlayerWorker::stopRecordingHandledSuccessfully,
            this,
            &QVideoPlayerWidget::onStopRecordingHandledSuccessfully);

    _ui.rtspTextBox->setText(QString::fromStdString(_camURL));
    _ui.rtspTextBox->setAlignment(Qt::AlignCenter);

    _ui.arucoIdsTextBox->setText("Ids: ");
}

void QVideoPlayerWidget::setArucoClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_)
{
    if (client_)
    {
        this->_client_arucoManager = client_;
    }

    else
    {
        _ui.arucoPushButton->setProperty("class", "error");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
}

void QVideoPlayerWidget::startDetection(void)
{
    if (_playerWorkerThread.get() != nullptr)
    {
        _playerWorkerThread->manageDetection(_client_arucoManager, _camURL, _tag, true);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, couldn't access Video Player worker");
    }
}

void QVideoPlayerWidget::stopDetection(void)
{
    if (_playerWorkerThread.get() != nullptr)
    {
        _playerWorkerThread->manageDetection(_client_arucoManager, _camURL, _tag, false);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, couldn't access Video Player worker");
    }
}

void QVideoPlayerWidget::handleArucoDetection(void)
{
    if (_ui.arucoPushButton->isChecked())
    {
        this->startDetection();

        if (!_ui.arucoPushButton->isChecked())
        {
            _ui.arucoPushButton->setChecked(true);
        }
        _ui.arucoPushButton->setProperty("class", "success");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
    else
    {
        this->stopDetection();
        if (_ui.arucoPushButton->isChecked())
        {
            _ui.arucoPushButton->setChecked(false);
        }
        _ui.arucoPushButton->setProperty("class", "normal");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
}

void QVideoPlayerWidget::arucoStillAliveUpdate(bool urlFound_)
{
    if (!urlFound_ && _ui.arucoPushButton->isChecked())
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error, aruco detection on %s was not found", _camURL.c_str());
        _ui.arucoPushButton->setProperty("class", "normal");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
}

void QVideoPlayerWidget::displayDetectedArucos(std::vector<uint16_t> ids_)
{
    size_t nbr_ids_detected = ids_.size();

    if (nbr_ids_detected > NBR_IDS_TO_DISPLAY)
    {
        ids_.resize(NBR_IDS_TO_DISPLAY);
    }
    _ui.arucoIdsTextBox->setText("Ids: ");

    for (const auto& id : ids_)
    {
        _ui.arucoIdsTextBox->setText(_ui.arucoIdsTextBox->text() + "  " + QString::number(id));
    }
}

void QVideoPlayerWidget::handlePlayPauseButton(void)
{
    if (_ui.playPauseButton->isChecked())
    {
        _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
    }
    else
    {
        _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-pause"));
    }
}

std::string QVideoPlayerWidget::getCamURL(void)
{
    return this->_camURL;
}

void QVideoPlayerWidget::setCamURL(std::string newCamUrl_)
{
    _camURL = newCamUrl_;
}

void QVideoPlayerWidget::setURLToDefault(void)
{
    _camURL = this->_defaultCamUrl;
    _ui.rtspTextBox->setText(QString::fromStdString(_camURL));
}

void QVideoPlayerWidget::updateCamURL()
{
    _camURL = _ui.rtspTextBox->text().toStdString();
}

void QVideoPlayerWidget::onDetectionHandledSuccessfully(bool success_, uint16_t tag_)
{
    if (!success_ && _tag == tag_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, request made on %s regarding aruco detection failed", _camURL.c_str());
        _ui.arucoPushButton->setProperty("class", "error");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
}

void QVideoPlayerWidget::onArucoServerInfoFailed(bool success_)
{
    if (!success_)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Error, info request to aruco detection manager client failed");
        _ui.arucoPushButton->setEnabled(false);
    }
    else
    {
        if (_ui.arucoPushButton->property("class") != "success" && _ui.arucoPushButton->property("class") != "error")
        {
            _ui.arucoPushButton->setProperty("class", "normal");
            _ui.arucoPushButton->setEnabled(true);
            _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
            _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
        }
    }
}

void QVideoPlayerWidget::onArucoCameraFailed(bool valid_)
{
    if (!valid_)
    {
        if (!_ui.arucoPushButton->isChecked())
        {
            _ui.arucoPushButton->setChecked(true);
        }
        if (!_ui.arucoPushButton->isEnabled())
        {
            _ui.arucoPushButton->setEnabled(true);
        }

        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, camera at %s is not accessible", _camURL.c_str());
        _ui.arucoPushButton->setProperty("class", "error");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }

    if (valid_)
    {
        if (!_ui.arucoPushButton->isChecked())
        {
            _ui.arucoPushButton->setChecked(true);
        }
        if (!_ui.arucoPushButton->isEnabled())
        {
            _ui.arucoPushButton->setEnabled(true);
        }
        _ui.arucoPushButton->setProperty("class", "success");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
}

void QVideoPlayerWidget::setCameraControlClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_)
{
    if (client_)
    {
        this->_client_cameraControlManager = client_;
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error, couldn't access aruco detection manager client");
        _ui.ScreenshotButton->setProperty("class", "error");
        _ui.ScreenshotButton->style()->unpolish(_ui.ScreenshotButton);
        _ui.ScreenshotButton->style()->polish(_ui.ScreenshotButton);
    }
}

void QVideoPlayerWidget::handleScreenshot(void)
{
    if (_playerWorkerThread.get() != nullptr)
    {
        _playerWorkerThread->takeScreenshotManager(_client_cameraControlManager, _camURL, _tag);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, couldn't access Video Player worker");
    }
    return;
}

void QVideoPlayerWidget::handleRecording(void)
{
    if (_playerWorkerThread.get() != nullptr)
    {
        if (_ui.startRecordingButton->isChecked())
        {
            _playerWorkerThread->startRecordingManager(_client_cameraControlManager, _camURL, _tag);
        }
        else
        {
            _playerWorkerThread->stopRecordingManager(_client_cameraControlManager, _camURL, _tag);
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, couldn't access Video Player worker");
    }
    return;
}

void QVideoPlayerWidget::onScreenshotHandledSuccessfully(bool success_, std::string status_, uint16_t tag_)
{
    if (tag_ == _tag)
    {
        if (!success_)
        {
            _ui.ScreenshotButton->setProperty("class", "error");
            _ui.ScreenshotButton->style()->unpolish(_ui.ScreenshotButton);
            _ui.ScreenshotButton->style()->polish(_ui.ScreenshotButton);
        }
        else
        {
            _ui.ScreenshotButton->setProperty("class", "success");
            _ui.ScreenshotButton->style()->unpolish(_ui.ScreenshotButton);
            _ui.ScreenshotButton->style()->polish(_ui.ScreenshotButton);
        }

        QTimer::singleShot(STYLE_RESET_TIME,
                           this,
                           [this]()
                           {
                               _ui.ScreenshotButton->setProperty("class", "normal");
                               _ui.ScreenshotButton->style()->unpolish(_ui.ScreenshotButton);
                               _ui.ScreenshotButton->style()->polish(_ui.ScreenshotButton);
                           });
    }
    return;
}

void QVideoPlayerWidget::onStartRecordingHandledSuccessfully(bool success_, std::string status_, uint16_t tag_)
{
    if (tag_ == _tag)
    {
        if (!success_)
        {
            _ui.startRecordingButton->setProperty("class", "error");
            _ui.startRecordingButton->style()->unpolish(_ui.startRecordingButton);
            _ui.startRecordingButton->style()->polish(_ui.startRecordingButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Screenshot taken", QString::fromStdString(status_), QHelper::QToastNotification::eNotifType::INFO);

            // reset after timer
            QTimer::singleShot(STYLE_RESET_TIME,
                               this,
                               [this]()
                               {
                                   _ui.startRecordingButton->setProperty("class", "");
                                   _ui.startRecordingButton->style()->unpolish(_ui.startRecordingButton);
                                   _ui.startRecordingButton->style()->polish(_ui.startRecordingButton);
                               });
        }
        else
        {
            _ui.startRecordingButton->setIcon(QIcon::fromTheme("media-playback-stop"));
            _ui.startRecordingButton->style()->unpolish(_ui.startRecordingButton);
            _ui.startRecordingButton->style()->polish(_ui.startRecordingButton);
        }
    }
    return;
}

void QVideoPlayerWidget::onStopRecordingHandledSuccessfully(bool success_, std::string status_, uint16_t tag_)
{
    if (tag_ == _tag)
    {
        if (!success_)
        {
            _ui.startRecordingButton->setProperty("class", "error");
            _ui.startRecordingButton->style()->unpolish(_ui.ScreenshotButton);
            _ui.startRecordingButton->style()->polish(_ui.ScreenshotButton);

            // reset after timer
            QTimer::singleShot(STYLE_RESET_TIME,
                               this,
                               [this]()
                               {
                                   _ui.startRecordingButton->setProperty("class", "");
                                   _ui.startRecordingButton->style()->unpolish(_ui.startRecordingButton);
                                   _ui.startRecordingButton->style()->polish(_ui.startRecordingButton);
                               });
        }
        else
        {
            _ui.startRecordingButton->setIcon(QIcon::fromTheme("media-record"));
            _ui.startRecordingButton->style()->unpolish(_ui.startRecordingButton);
            _ui.startRecordingButton->style()->polish(_ui.startRecordingButton);
        }
    }
    return;
}

void QVideoPlayerWidget::CB_cameraListUpdate(std::vector<std::string> urls)
{
    for (const auto& url : urls)
    {
        if (url == _camURL)
        {
            if (!_ui.startRecordingButton->isChecked())
            {
                _ui.startRecordingButton->clicked(true);
                _ui.startRecordingButton->setIcon(QIcon::fromTheme("media-playback-stop"));
                _ui.startRecordingButton->style()->unpolish(_ui.startRecordingButton);
                _ui.startRecordingButton->style()->polish(_ui.startRecordingButton);
            }
            return;
        }
    }

    // if cam_url wasn't found in vector and we're currently recording
    if (_ui.startRecordingButton->isChecked())
    {
        _ui.startRecordingButton->clicked(false);
        _ui.startRecordingButton->setIcon(QIcon::fromTheme("media-record"));
        _ui.startRecordingButton->style()->unpolish(_ui.startRecordingButton);
        _ui.startRecordingButton->style()->polish(_ui.startRecordingButton);
    }
}