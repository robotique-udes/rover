#include "QVideoPlayerWidget.hpp"
#include "QLogManager.hpp"
#include <QStyle>
#include <QDateTime>
#include <QMessageBox>
#include <QScrollBar>
#include <QRegularExpression>
#include <gst/video/videooverlay.h>

using namespace LogUtils;

int QVideoPlayerWidget::MAX_RECONNECT_ATTEMPTS = 3;
int QVideoPlayerWidget::_instanceCounter = 0;

QVideoPlayerWidget::QVideoPlayerWidget(std::shared_ptr<rclcpp::Node> guiNode_,
                                       std::string url_,
                                       uint16_t playerIndex_,
                                       std::shared_ptr<QPlayerWorker> workerThreadAruco_,
                                       std::shared_ptr<QPlayerWorker> workerThreadRecording_):
    _node(guiNode_),
    _camURL(url_),
    _tag(playerIndex_),
    _streamIndex(_instanceCounter - 1),
    _playerIndex(playerIndex_),
    _playerWorkerThreadAruco(workerThreadAruco_),
    _playerWorkerThreadRecording(workerThreadRecording_),
    _reconnectTimer(),
    _frameTimeoutTimer(),
    _connectionTimeoutTimer()
{
    _instanceCounter++;
    _defaultCamUrl = _camURL;
    _ui.setupUi(this);

    this->setupUI();

    _gstreamerWorker = new GStreamerWorker();
    _gstreamerWorker->setTargetWidget(_ui.logDisplay);

    _gstreamerWorker->moveToThread(&_gstreamerThread);

    connect(&_gstreamerThread, &QThread::finished, _gstreamerWorker, &QObject::deleteLater);
    connect(this, &QVideoPlayerWidget::requestStartStream, _gstreamerWorker, &GStreamerWorker::startPipeline);
    connect(this, &QVideoPlayerWidget::requestStopStream, _gstreamerWorker, &GStreamerWorker::stopPipeline);
    connect(_gstreamerWorker, &GStreamerWorker::pipelineStarted, this, &QVideoPlayerWidget::onPipelineStarted);
    connect(_gstreamerWorker, &GStreamerWorker::errorOccurred, this, &QVideoPlayerWidget::onErrorOccurred);
    connect(_gstreamerWorker, &GStreamerWorker::connectionFailed, this, &QVideoPlayerWidget::onConnectionFailed);
    connect(_gstreamerWorker, &GStreamerWorker::frameReceived, this, &QVideoPlayerWidget::onFrameReceived);
    this->hideAngleSelecter();

    connect(_ui.arucoPushButton, &QPushButton::clicked, this, &QVideoPlayerWidget::handleArucoDetection);
    connect(_playerWorkerThreadAruco.get(),
            &QPlayerWorker::detectionHandledSuccessfully,
            this,
            &QVideoPlayerWidget::onDetectionHandledSuccessfully);
    connect(_ui.playPauseButton, &QPushButton::clicked, this, &QVideoPlayerWidget::handlePlayPauseButton);
    connect(_playerWorkerThreadAruco.get(),
            &QPlayerWorker::arucoServerInfoFailed,
            this,
            &QVideoPlayerWidget::onArucoServerInfoFailed);

    connect(_ui.rtspTextBox, &QLineEdit::textChanged, this, &QVideoPlayerWidget::updateCamURL);
    connect(_ui.defaultStreamPushButton, &QPushButton::clicked, this, &QVideoPlayerWidget::setURLToDefault);
    connect(this, &QVideoPlayerWidget::arucoCameraFailure, this, &QVideoPlayerWidget::onArucoCameraFailed);

    connect(&_frameTimeoutTimer, &QTimer::timeout, this, &QVideoPlayerWidget::onFrameTimeout);
    connect(&_reconnectTimer, &QTimer::timeout, this, &QVideoPlayerWidget::onReconnectTimer);
    connect(&_connectionTimeoutTimer, &QTimer::timeout, this, &QVideoPlayerWidget::onConnectionTimeout);

    connect(&QLogManager::getInstance(), &QLogManager::newLogMessage, this, &QVideoPlayerWidget::onNewLogMessage);

    connect(_ui.ScreenshotButton, &QPushButton::clicked, this, &QVideoPlayerWidget::handleScreenshot);
    connect(_playerWorkerThreadRecording.get(),
            &QPlayerWorker::screenshotHandledSuccessfully,
            this,
            &QVideoPlayerWidget::onScreenshotHandledSuccessfully);

    connect(_ui.startRecordingButton, &QPushButton::clicked, this, &QVideoPlayerWidget::handleRecording);
    connect(_ui.cameraAngleSlider, &QSlider::valueChanged, this, &QVideoPlayerWidget::onCameraAngleSliderChanged);
    connect(_ui.cameraAngleBox, &QDoubleSpinBox::valueChanged, this, &QVideoPlayerWidget::onCameraAngleBoxChanged);

    connect(_playerWorkerThreadRecording.get(),
            &QPlayerWorker::startRecordingHandledSuccessfully,
            this,
            &QVideoPlayerWidget::onStartRecordingHandledSuccessfully);

    connect(_playerWorkerThreadRecording.get(),
            &QPlayerWorker::stopRecordingHandledSuccessfully,
            this,
            &QVideoPlayerWidget::onStopRecordingHandledSuccessfully);

    _ui.rtspTextBox->setText(QString::fromStdString(_camURL));
    _ui.rtspTextBox->setAlignment(Qt::AlignCenter);
    _ui.arucoIdsTextBox->setText("Ids: ");

    this->setPlayerState(ePlayerState::NOT_CONNECTED);

    _gstreamerThread.start();

    UI_LOG_INFO(GENERAL, QString::fromStdString("VideoPlayer Widget initialized for camera: " + _camURL), _ui.logDisplay);
}

QVideoPlayerWidget::~QVideoPlayerWidget()
{
    this->cleanupResources();
}

void QVideoPlayerWidget::cleanupResources(void)
{
    this->stopStream();

    if (_gstreamerThread.isRunning())
    {
        _gstreamerThread.quit();
        if (!_gstreamerThread.wait(1000))
        {
            _gstreamerThread.terminate();
            _gstreamerThread.wait();
        }
    }
}

void QVideoPlayerWidget::setupUI(void)
{
    this->connectUISignals();
    this->initializeUIState();

    _ui.videoWidget->setStyleSheet("background-color: black;");
    _ui.videoWidget->setVisible(true);
    _ui.stackedWidget->setCurrentIndex(0);
    _ui.logDisplay->clear();
    _ui.logDisplay->setAcceptRichText(true);
    _ui.logDisplay->document()->setDefaultStyleSheet(
        "span.debug { color: gray; } span.info { color: white; } span.warning { color: orange; } span.error { color: #FF5555; }");
    _ui.logDisplay->setStyleSheet("font-family: monospace; background-color: #222222;");
}

void QVideoPlayerWidget::connectUISignals(void)
{
    connect(_ui.playPauseButton, &QPushButton::clicked, this, &QVideoPlayerWidget::handlePlayPauseButton);
    connect(_ui.toggleViewButton, &QPushButton::clicked, this, &QVideoPlayerWidget::onToggleView);
    connect(_ui.rtspTextBox, &QLineEdit::textChanged, this, &QVideoPlayerWidget::onUrlTextChanged);

    QPushButton* backToVideoBtn = this->findChild<QPushButton*>("backToVideoBtn");
    if (backToVideoBtn)
    {
        connect(backToVideoBtn,
                &QPushButton::clicked,
                this,
                [this]()
                {
                    _ui.stackedWidget->setCurrentIndex(0);
                });
    }

    QPushButton* clearLogsBtn = findChild<QPushButton*>("clearLogsBtn");
    if (clearLogsBtn)
    {
        connect(clearLogsBtn, &QPushButton::clicked, this, &QVideoPlayerWidget::clearLogs);
    }

    QCheckBox* debugCheckbox = findChild<QCheckBox*>("_debugCheckbox");
    if (debugCheckbox)
    {
        connect(debugCheckbox,
                &QCheckBox::toggled,
                this,
                [this](bool checked)
                {
                    QLogManager::getInstance().setShowDebug(checked, _ui.logDisplay);
                });
    }

    QCheckBox* infoCheckbox = findChild<QCheckBox*>("_infoCheckbox");
    if (infoCheckbox)
    {
        connect(infoCheckbox,
                &QCheckBox::toggled,
                this,
                [this](bool checked)
                {
                    QLogManager::getInstance().setShowInfo(checked, _ui.logDisplay);
                });
    }

    QCheckBox* warningCheckbox = findChild<QCheckBox*>("_warningCheckbox");
    if (warningCheckbox)
    {
        connect(warningCheckbox,
                &QCheckBox::toggled,
                this,
                [this](bool checked)
                {
                    QLogManager::getInstance().setShowWarning(checked, _ui.logDisplay);
                });
    }

    QCheckBox* errorCheckbox = findChild<QCheckBox*>("_errorCheckbox");
    if (errorCheckbox)
    {
        connect(errorCheckbox,
                &QCheckBox::toggled,
                this,
                [this](bool checked)
                {
                    QLogManager::getInstance().setShowError(checked, _ui.logDisplay);
                });
    }
}

void QVideoPlayerWidget::initializeUIState(void)
{
    this->updateStatusText("Not Connected");
    this->_controlsVisible = true;

    _ui.playPauseButton->setChecked(false);
    _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
    _ui.arucoPushButton->setEnabled(false);
    _ui.ScreenshotButton->setEnabled(false);
    _ui.startRecordingButton->setEnabled(false);

    QCheckBox* debugCheckbox = findChild<QCheckBox*>("_debugCheckbox");
    if (debugCheckbox)
    {
        debugCheckbox->setChecked(false);
    }

    QCheckBox* infoCheckbox = findChild<QCheckBox*>("_infoCheckbox");
    if (infoCheckbox)
    {
        infoCheckbox->setChecked(true);
    }

    QCheckBox* warningCheckbox = findChild<QCheckBox*>("_warningCheckbox");
    if (warningCheckbox)
    {
        warningCheckbox->setChecked(true);
    }

    QCheckBox* errorCheckbox = findChild<QCheckBox*>("_errorCheckbox");
    if (errorCheckbox)
    {
        errorCheckbox->setChecked(true);
    }
}

void QVideoPlayerWidget::onNewLogMessage(const QString& message_, QWidget* targetWidget_)
{
    if (targetWidget_ == _ui.logDisplay)
    {
        _ui.logDisplay->append(message_);

        QScrollBar* scrollBar = _ui.logDisplay->verticalScrollBar();
        if (scrollBar)
        {
            scrollBar->setValue(scrollBar->maximum());
        }
    }
}

void QVideoPlayerWidget::startStream(const QString& rtspUrl_)
{
    if (rtspUrl_.isEmpty())
    {
        UI_LOG_WARNING_RTSP("Empty RTSP URL provided", _ui.logDisplay);
        return;
    }

    if (!this->validateRtspUrl(rtspUrl_))
    {
        UI_LOG_WARNING_RTSP(QString("Invalid RTSP URL: %1").arg(rtspUrl_), _ui.logDisplay);
        QMessageBox::warning(this,
                             "Invalid RTSP URL",
                             "The URL format is invalid. Please enter a valid RTSP URL.\n\n"
                             "Format: rtsp://[username:password@]host[:port]/path");
        return;
    }

    QDateTime currentTime = QDateTime::currentDateTime();
    if (rtspUrl_ == QString::fromStdString(_camURL) && _lastStreamTime.isValid() && _lastStreamTime.msecsTo(currentTime) < 500)
    {
        return;
    }

    _lastStreamTime = currentTime;
    _camURL = rtspUrl_.toStdString();

    if (_state != ePlayerState::RECONNECTING)
    {
        UI_LOG_INFO_RTSP(QString("Starting stream: %1").arg(rtspUrl_), _ui.logDisplay);
        _reconnectAttempts = 0;
    }

    this->setPlayerState(ePlayerState::CONNECTING);

    emit requestStartStream(rtspUrl_);
}

void QVideoPlayerWidget::stopStream(void)
{
    _connectionTimeoutTimer.stop();

    if (_state == ePlayerState::NOT_CONNECTED || _state == ePlayerState::PAUSED)
    {
        return;
    }

    UI_LOG_INFO_RTSP(QString("Stopping stream: %1").arg(QString::fromStdString(_camURL)), _ui.logDisplay);

    _frameTimeoutTimer.stop();
    _reconnectTimer.stop();

    _ui.arucoPushButton->setEnabled(false);
    _ui.ScreenshotButton->setEnabled(false);
    _ui.startRecordingButton->setEnabled(false);

    emit requestStopStream();

    if (_wasEverConnected)
    {
        this->setPlayerState(ePlayerState::PAUSED);
    }
    else
    {
        this->setPlayerState(ePlayerState::NOT_CONNECTED);
    }
}

void QVideoPlayerWidget::setPlayerState(ePlayerState state_)
{
    if (_state == state_)
    {
        return;
    }

    ePlayerState oldState = _state;
    _state = state_;

    switch (_state)
    {
        case ePlayerState::NOT_CONNECTED:
            this->updateStatusText("Not Connected");
            _ui.playPauseButton->setChecked(false);
            _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            break;

        case ePlayerState::CONNECTING:
            this->updateStatusText("Connecting...");
            _ui.playPauseButton->setChecked(true);
            _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-pause"));
            _connectionTimeoutTimer.start(8000);
            break;

        case ePlayerState::STREAMING:
            this->updateStatusText("");
            _ui.playPauseButton->setChecked(true);
            _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-pause"));
            _wasEverConnected = true;
            _ui.arucoPushButton->setEnabled(true);
            _frameTimeoutTimer.start(2000);
            _ui.ScreenshotButton->setEnabled(true);
            _ui.startRecordingButton->setEnabled(true);
            UI_LOG_INFO_RTSP("Stream connected successfully", _ui.logDisplay);
            break;

        case ePlayerState::RECONNECTING:
            this->updateStatusText(QString("Reconnecting... (%1/%2)").arg(_reconnectAttempts).arg(MAX_RECONNECT_ATTEMPTS));
            _ui.playPauseButton->setChecked(false);
            _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            if (_ui.arucoPushButton->isChecked())
            {
                UI_LOG_INFO_RTSP("Resetting Aruco button due to stream loss", _ui.logDisplay);
                _ui.arucoPushButton->setChecked(false);
                _ui.arucoPushButton->setProperty("class", "normal");
                _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
                _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
                _ui.arucoIdsTextBox->setText("Ids: ");
            }
            _ui.arucoPushButton->setEnabled(false);
            _ui.ScreenshotButton->setEnabled(false);
            _ui.startRecordingButton->setEnabled(false);
            break;

        case ePlayerState::PAUSED:
            this->updateStatusText("Paused");
            _ui.playPauseButton->setChecked(false);
            _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            _ui.arucoPushButton->setEnabled(false);
            _ui.ScreenshotButton->setEnabled(false);
            _ui.startRecordingButton->setEnabled(false);
            break;

        case ePlayerState::CONNECTION_ERROR:
            this->updateStatusText("Connection Error");
            _ui.playPauseButton->setChecked(false);
            _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            UI_LOG_ERROR_RTSP("Connection error occurred", _ui.logDisplay);
            this->tryReconnect();
            break;

        case ePlayerState::CONNECTION_FAILED:
            this->updateStatusText("Connection Failed");
            _ui.playPauseButton->setChecked(false);
            _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            _ui.arucoPushButton->setEnabled(false);
            _ui.ScreenshotButton->setEnabled(false);
            _ui.startRecordingButton->setEnabled(false);
            UI_LOG_ERROR_RTSP("Connection failed permanently", _ui.logDisplay);
            break;
    }

    bool wasStreaming = (oldState == ePlayerState::STREAMING);
    bool isStreaming = (_state == ePlayerState::STREAMING);

    if (wasStreaming != isStreaming)
    {
        this->emitStateChanged();
    }
}

void QVideoPlayerWidget::tryReconnect(void)
{
    if (_state == ePlayerState::CONNECTION_FAILED)
    {
        return;
    }

    _reconnectAttempts++;

    if (_reconnectAttempts <= MAX_RECONNECT_ATTEMPTS)
    {
        UI_LOG_INFO_RTSP(QString("Automatic reconnection attempt %1 of %2").arg(_reconnectAttempts).arg(MAX_RECONNECT_ATTEMPTS),
                         _ui.logDisplay);
        this->setPlayerState(ePlayerState::RECONNECTING);
        _reconnectTimer.start(3000);
    }
    else
    {
        UI_LOG_ERROR_RTSP("Maximum reconnection attempts reached", _ui.logDisplay);
        this->setPlayerState(ePlayerState::CONNECTION_FAILED);
    }
}

void QVideoPlayerWidget::updateStatusText(const QString& text_)
{
    _ui.statusLabel->setText(text_);
    _ui.videoStack->setCurrentIndex(text_.isEmpty() ? 0 : 1);
}

bool QVideoPlayerWidget::validateRtspUrl(const QString& url_)
{
    return url_.startsWith("rtsp://") || url_.startsWith("rtspt://") || url_.startsWith("rtsps://");
}

void QVideoPlayerWidget::updateUrlValidationUI(bool isValid_)
{
    if (isValid_)
    {
        _ui.rtspTextBox->setStyleSheet("");
        _ui.rtspTextBox->setToolTip("");
    }
    else
    {
        _ui.rtspTextBox->setStyleSheet("border: 1px solid red;");
        _ui.rtspTextBox->setToolTip("Invalid URL format. Expected: rtsp://[username:password@]host[:port]/path");
    }
}

void QVideoPlayerWidget::emitStateChanged(void)
{
    emit streamStateChanged(_state == ePlayerState::STREAMING, _streamIndex);
}

void QVideoPlayerWidget::onToggleView(void)
{
    int currentIndex = _ui.stackedWidget->currentIndex();
    int newIndex = (currentIndex == 0) ? 1 : 0;
    _ui.stackedWidget->setCurrentIndex(newIndex);
}

void QVideoPlayerWidget::onUrlTextChanged(const QString& text_)
{
    if (text_.isEmpty())
    {
        _ui.rtspTextBox->setStyleSheet("");
        _ui.rtspTextBox->setToolTip("Enter RTSP URL...");
    }
    else
    {
        bool isValid = this->validateRtspUrl(text_);
        this->updateUrlValidationUI(isValid);

        if (_state == ePlayerState::CONNECTION_FAILED && isValid)
        {
            this->setPlayerState(ePlayerState::NOT_CONNECTED);
        }
    }
}

void QVideoPlayerWidget::clearLogs(void)
{
    _ui.logDisplay->clear();
    UI_LOG_INFO_RTSP("Logs cleared", _ui.logDisplay);
}

void QVideoPlayerWidget::toggleLogView(bool show_)
{
    _ui.stackedWidget->setCurrentIndex(show_ ? 1 : 0);
}

void QVideoPlayerWidget::onPipelineStarted(GstElement* pipeline_)
{
    if (!pipeline_)
    {
        UI_LOG_ERROR_RTSP("Pipeline creation failed", _ui.logDisplay);
        _connectionTimeoutTimer.stop();
        this->setPlayerState(ePlayerState::CONNECTION_ERROR);
        return;
    }

    if (_pipeline != nullptr)
    {
        UI_LOG_DEBUG_RTSP("Replacing existing pipeline reference", _ui.logDisplay);
    }

    _pipeline = pipeline_;
    GstElement* videoSink = gst_bin_get_by_interface(GST_BIN(_pipeline), GST_TYPE_VIDEO_OVERLAY);

    if (!videoSink)
    {
        UI_LOG_ERROR_RTSP("Failed to find VideoOverlay in pipeline", _ui.logDisplay);
        _connectionTimeoutTimer.stop();
        this->setPlayerState(ePlayerState::CONNECTION_ERROR);
        return;
    }

    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(videoSink), (guintptr)_ui.videoWidget->winId());
    gst_object_unref(videoSink);

    gst_element_set_state(_pipeline, GST_STATE_PLAYING);
    UI_LOG_DEBUG_RTSP("Pipeline state set to PLAYING", _ui.logDisplay);

    _frameTimeoutTimer.start(2000);
}

void QVideoPlayerWidget::onErrorOccurred(const QString& error_)
{
    if (_state == ePlayerState::STREAMING)
    {
        return;
    }

    if (_state == ePlayerState::RECONNECTING)
    {
        UI_LOG_DEBUG_RTSP("Stream error: " + error_, _ui.logDisplay);

        if (!_reconnectTimer.isActive())
        {
            _reconnectTimer.start(3000);
        }
    }
    else
    {
        UI_LOG_ERROR_RTSP("Stream error: " + error_, _ui.logDisplay);
        this->setPlayerState(ePlayerState::CONNECTION_ERROR);
    }
}

void QVideoPlayerWidget::onConnectionFailed(void)
{
    _reconnectTimer.stop();
    _connectionTimeoutTimer.stop();
    _reconnectAttempts = 0;
    this->setPlayerState(ePlayerState::CONNECTION_FAILED);
    UI_LOG_ERROR_RTSP("Connection failed permanently", _ui.logDisplay);
}

void QVideoPlayerWidget::onFrameReceived(void)
{
    _connectionTimeoutTimer.stop();
    _reconnectTimer.stop();

    if (_state != ePlayerState::STREAMING)
    {
        if (_state == ePlayerState::RECONNECTING)
        {
            UI_LOG_INFO_RTSP("Reconnection successful, receiving frames...", _ui.logDisplay);
            _reconnectAttempts = 0;
        }
        else
        {
            UI_LOG_INFO_RTSP("Receiving frames...", _ui.logDisplay);
        }

        this->setPlayerState(ePlayerState::STREAMING);

        _ui.arucoPushButton->setEnabled(true);
        _ui.ScreenshotButton->setEnabled(true);
        _ui.startRecordingButton->setEnabled(true);
    }
    else
    {
        _frameTimeoutTimer.start(2000);
    }
}

void QVideoPlayerWidget::onFrameTimeout(void)
{
    if (_state == ePlayerState::STREAMING)
    {
        UI_LOG_WARNING_RTSP("Frame timeout - no frames received", _ui.logDisplay);

        _ui.arucoPushButton->setEnabled(false);
        _ui.ScreenshotButton->setEnabled(false);
        _ui.startRecordingButton->setEnabled(false);

        if (_ui.arucoPushButton->isChecked())
        {
            UI_LOG_INFO_RTSP("Resetting Aruco button due to frame timeout", _ui.logDisplay);
            _ui.arucoPushButton->setChecked(false);
            _ui.arucoPushButton->setProperty("class", "normal");
            _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
            _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
            _ui.arucoIdsTextBox->setText("Ids: ");
        }

        this->setPlayerState(ePlayerState::CONNECTION_ERROR);
    }
}

void QVideoPlayerWidget::onReconnectTimer(void)
{
    if (_state == ePlayerState::RECONNECTING)
    {
        if (!_ui.rtspTextBox->text().isEmpty())
        {
            this->startStream(_ui.rtspTextBox->text());
        }
    }
}

void QVideoPlayerWidget::onConnectionTimeout(void)
{
    UI_LOG_ERROR_RTSP("Connection timeout - no response from server", _ui.logDisplay);

    if (_ui.arucoPushButton->isChecked())
    {
        UI_LOG_INFO_RTSP("Resetting Aruco button due to connection timeout", _ui.logDisplay);
        _ui.arucoPushButton->setChecked(false);
        _ui.arucoPushButton->setProperty("class", "normal");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
        _ui.arucoIdsTextBox->setText("Ids: ");
    }

    _reconnectAttempts = 0;
    this->setPlayerState(ePlayerState::CONNECTION_FAILED);
    emit requestStopStream();
}

void QVideoPlayerWidget::handlePlayPauseButton(void)
{
    if (!_ui.playPauseButton->isChecked())
    {
        _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
        this->stopStream();
    }
    else
    {
        _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-pause"));
        this->startStream(QString::fromStdString(_camURL));
    }
}

void QVideoPlayerWidget::setArucoClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_)
{
    if (client_)
    {
        this->_client_arucoManager = client_;
    }
    else
    {
        UI_LOG_WARNING(ARUCO_DETECTION, "Error, couldn't access aruco detection manager client", _ui.logDisplay);
        _ui.arucoPushButton->setProperty("class", "error");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
}

void QVideoPlayerWidget::startDetection(void)
{
    if (_playerWorkerThreadAruco.get() != nullptr)
    {
        _playerWorkerThreadAruco->manageDetection(_client_arucoManager, _camURL, _playerIndex, true);
    }
    else
    {
        UI_LOG_ERROR(ARUCO_DETECTION, "Error, couldn't access Video Player worker", _ui.logDisplay);
    }
}

void QVideoPlayerWidget::stopDetection(void)
{
    if (_playerWorkerThreadAruco.get() != nullptr)
    {
        _playerWorkerThreadAruco->manageDetection(_client_arucoManager, _camURL, _playerIndex, false);
    }
    else
    {
        UI_LOG_ERROR(ARUCO_DETECTION, "Error, couldn't access Video Player worker", _ui.logDisplay);
    }
}

void QVideoPlayerWidget::handleArucoDetection(void)
{
    if (_ui.arucoPushButton->isChecked())
    {
        UI_LOG_INFO(ARUCO_DETECTION,
                    QString("Starting aruco detection on camera %1").arg(QString::fromStdString(_camURL)),
                    _ui.logDisplay);
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
        UI_LOG_INFO(ARUCO_DETECTION,
                    QString("Stopping aruco detection on camera %1").arg(QString::fromStdString(_camURL)),
                    _ui.logDisplay);
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
        UI_LOG_WARNING(ARUCO_DETECTION,
                       QString("Error, aruco detection on %1 was not found").arg(QString::fromStdString(_camURL)),
                       _ui.logDisplay);
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

    if (!ids_.empty())
    {
        UI_LOG_INFO(ARUCO_DETECTION,
                    QString("Detected aruco markers on camera %1: %2")
                        .arg(QString::fromStdString(_camURL))
                        .arg(_ui.arucoIdsTextBox->text().mid(5)),
                    _ui.logDisplay);
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

float QVideoPlayerWidget::getCameraAngle(void)
{
    return static_cast<float>(_ui.cameraAngleSlider->value());
}

void QVideoPlayerWidget::setCamURL(std::string newCamUrl_)
{
    _camURL = newCamUrl_;
    this->hideAngleSelecter();
}

void QVideoPlayerWidget::setURLToDefault(void)
{
    _camURL = this->_defaultCamUrl;
    _ui.rtspTextBox->setText(QString::fromStdString(_camURL));
    this->hideAngleSelecter();
}

void QVideoPlayerWidget::updateCamURL()
{
    _camURL = _ui.rtspTextBox->text().toStdString();
    this->hideAngleSelecter();
}

void QVideoPlayerWidget::onDetectionHandledSuccessfully(bool success_, uint16_t playerIndex_)
{
    if (!success_ && _playerIndex == playerIndex_)
    {
        UI_LOG_ERROR(ARUCO_DETECTION,
                     QString("Error, request made on %1 regarding aruco detection failed").arg(QString::fromStdString(_camURL)),
                     _ui.logDisplay);
        _ui.arucoPushButton->setProperty("class", "error");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
}

void QVideoPlayerWidget::onArucoServerInfoFailed(bool success_)
{
    if (!success_)
    {
        UI_LOG_DEBUG(ARUCO_DETECTION, "Error, info request to aruco detection manager client failed", _ui.logDisplay);
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

        UI_LOG_ERROR(ARUCO_DETECTION,
                     QString("Error, camera at %1 is not accessible").arg(QString::fromStdString(_camURL)),
                     _ui.logDisplay);
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

QString QVideoPlayerWidget::getId(void)
{
    return QString("widget_%1").arg(reinterpret_cast<quintptr>(_ui.logDisplay));
}

bool QVideoPlayerWidget::isStreaming(void)
{
    return _state == ePlayerState::STREAMING;
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
    if (_playerWorkerThreadRecording.get() != nullptr)
    {
        _playerWorkerThreadRecording->takeScreenshotManager(_client_cameraControlManager, _camURL, _playerIndex);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, couldn't access Video Player worker");
    }
    return;
}

void QVideoPlayerWidget::handleRecording(void)
{
    if (_playerWorkerThreadRecording.get() != nullptr)
    {
        if (_ui.startRecordingButton->isChecked())
        {
            _playerWorkerThreadRecording->startRecordingManager(_client_cameraControlManager, _camURL, _playerIndex);
        }
        else
        {
            _playerWorkerThreadRecording->stopRecordingManager(_client_cameraControlManager, _camURL, _playerIndex);
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, couldn't access Video Player worker");
    }
    return;
}

void QVideoPlayerWidget::onScreenshotHandledSuccessfully(bool success_, std::string status_, uint16_t playerIndex_)
{
    if (playerIndex_ == _playerIndex)
    {
        if (!success_)
        {
            _ui.ScreenshotButton->setProperty("class", "error");
            _ui.ScreenshotButton->style()->unpolish(_ui.ScreenshotButton);
            _ui.ScreenshotButton->style()->polish(_ui.ScreenshotButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Couldn't take screenshot",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::ERROR);
        }
        else
        {
            _ui.ScreenshotButton->setProperty("class", "success");
            _ui.ScreenshotButton->style()->unpolish(_ui.ScreenshotButton);
            _ui.ScreenshotButton->style()->polish(_ui.ScreenshotButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Screenshot taken",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::INFO);
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

void QVideoPlayerWidget::onStartRecordingHandledSuccessfully(bool success_, std::string status_, uint16_t playerIndex_)
{
    if (playerIndex_ == _playerIndex)
    {
        if (!success_)
        {
            _ui.startRecordingButton->setProperty("class", "error");
            _ui.startRecordingButton->style()->unpolish(_ui.startRecordingButton);
            _ui.startRecordingButton->style()->polish(_ui.startRecordingButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Couldn't start video",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::ERROR);

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
            _ui.startRecordingButton->setProperty("class", "success");
            _ui.startRecordingButton->setIcon(QIcon::fromTheme("media-playback-stop"));
            _ui.startRecordingButton->style()->unpolish(_ui.startRecordingButton);
            _ui.startRecordingButton->style()->polish(_ui.startRecordingButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Video started",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::INFO);
        }
    }
    return;
}

void QVideoPlayerWidget::onStopRecordingHandledSuccessfully(bool success_, std::string status_, uint16_t playerIndex_)
{
    if (playerIndex_ == _playerIndex)
    {
        if (!success_)
        {
            _ui.startRecordingButton->setProperty("class", "error");
            _ui.startRecordingButton->style()->unpolish(_ui.ScreenshotButton);
            _ui.startRecordingButton->style()->polish(_ui.ScreenshotButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Couldn't stop recording",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::ERROR);

            // reset after timer
            QTimer::singleShot(STYLE_RESET_TIME,
                               this,
                               [this]()
                               {
                                   _ui.startRecordingButton->setProperty("class", "normal");
                                   _ui.startRecordingButton->style()->unpolish(_ui.startRecordingButton);
                                   _ui.startRecordingButton->style()->polish(_ui.startRecordingButton);
                               });
        }
        else
        {
            _ui.startRecordingButton->setProperty("class", "normal");
            _ui.startRecordingButton->setIcon(QIcon::fromTheme("media-record"));
            _ui.startRecordingButton->style()->unpolish(_ui.startRecordingButton);
            _ui.startRecordingButton->style()->polish(_ui.startRecordingButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Recording stopped",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::INFO);
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
                _ui.startRecordingButton->setChecked(true);
                _ui.startRecordingButton->setProperty("class", "success");
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
        std::string error_message = "Recording on " + _camURL + " was stopped unexpectedly";
        _ui.startRecordingButton->setChecked(false);
        _ui.startRecordingButton->setIcon(QIcon::fromTheme("media-record"));
        _ui.startRecordingButton->style()->unpolish(_ui.startRecordingButton);
        _ui.startRecordingButton->style()->polish(_ui.startRecordingButton);
        QHelper::QToastNotification::getInstance().notifyFromAnyThread("Recording stopped",
                                                                       error_message,
                                                                       QHelper::QToastNotification::eNotifType::WARNING);
    }
}

void QVideoPlayerWidget::CB_serviceCameraControlAvailable(bool available_)
{
    if (!available_)
    {
        _ui.ScreenshotButton->setEnabled(false);
        _ui.startRecordingButton->setEnabled(false);
        if (_playerIndex == 1)
        {
            RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("GUI"),
                                  *_node->get_clock(),
                                  THROTTLE_RATE_ERROR,
                                  "Error, camera control client is unavailable ");
        }
    }
    else if (!_ui.ScreenshotButton->isEnabled() || !_ui.startRecordingButton->isEnabled())
    {
        _ui.ScreenshotButton->setEnabled(true);
        _ui.startRecordingButton->setEnabled(true);
    }
}

void QVideoPlayerWidget::onCameraAngleSliderChanged(void)
{
    _ui.cameraAngleBox->setValue(_ui.cameraAngleSlider->value());
    float angle = static_cast<float>(_ui.cameraAngleSlider->value());
    emit this->notifyCameraAnglePublisher(_camURL, angle);
}

void QVideoPlayerWidget::onCameraAngleBoxChanged(void)
{
    _ui.cameraAngleSlider->setValue(_ui.cameraAngleBox->value());
    float angle = static_cast<float>(_ui.cameraAngleBox->value());
    emit this->notifyCameraAnglePublisher(_camURL, angle);
}

void QVideoPlayerWidget::hideAngleSelecter(void)
{
    if (_camURL == Constants::CameraInfo::CAMERA_URL_MAP.at("Main")
        || _camURL == Constants::CameraInfo::CAMERA_URL_MAP.at("Antenna"))
    {
        _ui.cameraAngleSlider->show();
        _ui.cameraAngleBox->show();
    }
    else
    {
        _ui.cameraAngleSlider->hide();
        _ui.cameraAngleBox->hide();
    }
}