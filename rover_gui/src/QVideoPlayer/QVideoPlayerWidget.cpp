#include "QVideoPlayerWidget.hpp"
#include "QLogManager.hpp"
#include <Global/Helpers/QSessionFolderManager/QSessionFolderManager.hpp>
#include <QStyle>
#include <QDateTime>
#include <QMessageBox>
#include <QScrollBar>
#include <QRegularExpression>
#include <optional>
#include <gst/video/videooverlay.h>

using namespace LogUtils;

int QVideoPlayerWidget::MAX_RECONNECT_ATTEMPTS = 3;
int QVideoPlayerWidget::_instanceCounter = 0;

QVideoPlayerWidget::QVideoPlayerWidget(std::shared_ptr<rclcpp::Node> guiNode_,
                                       std::string url_,
                                       uint16_t playerIndex_,
                                       std::shared_ptr<QPlayerWorker> workerThreadAruco_,
                                       std::shared_ptr<QRecordingWorker> workerThreadRecording_,
                                       std::shared_ptr<QPanoramaWorker> workerThreadPanorama_):
    _node(guiNode_),
    _camURL(url_),
    _streamIndex(_instanceCounter - 1),
    _playerIndex(playerIndex_),
    _playerWorkerThreadAruco(workerThreadAruco_),
    _playerWorkerThreadRecording(workerThreadRecording_),
    _panoramaWorkerThread(workerThreadPanorama_),
    _recorderWidget(url_, playerIndex_, workerThreadRecording_),
    _reconnectTimer(),
    _frameTimeoutTimer(),
    _connectionTimeoutTimer()

{
    _instanceCounter++;
    _defaultCamUrl = _camURL;
    _ui.setupUi(this);

    this->setupUI();

    sRecordingButtons recordingButtons = {_ui.startRecordingButton, _ui.ScreenshotButton};
    _recorderWidget.setButtons(recordingButtons);

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
    this->hideAngleSelector();

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

    connect(this, &QVideoPlayerWidget::displayDetectedArucos, this, &QVideoPlayerWidget::onDisplayDetectedArucos);

    connect(_ui.rtspTextBox, &QLineEdit::textChanged, this, &QVideoPlayerWidget::updateCamURL);
    connect(_ui.defaultStreamPushButton, &QPushButton::clicked, this, &QVideoPlayerWidget::setURLToDefault);
    connect(this, &QVideoPlayerWidget::arucoCameraFailure, this, &QVideoPlayerWidget::onArucoCameraFailed);

    connect(&_frameTimeoutTimer, &QTimer::timeout, this, &QVideoPlayerWidget::onFrameTimeout);
    connect(&_reconnectTimer, &QTimer::timeout, this, &QVideoPlayerWidget::onReconnectTimer);
    connect(&_connectionTimeoutTimer, &QTimer::timeout, this, &QVideoPlayerWidget::onConnectionTimeout);

    connect(&QLogManager::getInstance(), &QLogManager::newLogMessage, this, &QVideoPlayerWidget::onNewLogMessage);

    connect(_ui.cameraAngleSlider, &QSlider::valueChanged, this, &QVideoPlayerWidget::onCameraAngleSliderChanged);
    connect(_ui.cameraAngleBox, &QDoubleSpinBox::valueChanged, this, &QVideoPlayerWidget::onCameraAngleBoxChanged);

    connect(_ui.angleCenterButton, &QPushButton::clicked, this, &QVideoPlayerWidget::onCenterAngle);
    connect(_ui.panoramaButton, &QPushButton::clicked, this, &QVideoPlayerWidget::handlePanorama);
    connect(_panoramaWorkerThread.get(), &QPanoramaWorker::panoramaStarted, this, &QVideoPlayerWidget::onPanoramaStarted);
    connect(_panoramaWorkerThread.get(), &QPanoramaWorker::panoramaFinished, this, &QVideoPlayerWidget::onPanoramaFinished);
    connect(_ui.panoramaDurationBox, &QDoubleSpinBox::valueChanged, this, &QVideoPlayerWidget::setPanoramaDuration);
    connect(this, &QVideoPlayerWidget::updateActualAngle, this, &QVideoPlayerWidget::onUpdateActualAngle);
    _ui.rtspTextBox->setText(QString::fromStdString(_camURL));
    _ui.rtspTextBox->setAlignment(Qt::AlignCenter);
    _ui.arucoIdsTextBox->setText("Ids: ");
    _ui.cameraAngleSlider->setValue(CAMERA_CENTER_ANGLE);
    _ui.cameraAngleBox->setValue(CAMERA_CENTER_ANGLE);

    this->setPlayerState(ePlayerState::NOT_CONNECTED);

    _gstreamerThread.start();
    this->autoStartGStreamer();

    // In the next implementation of panorama move this to QPanoramaHandler
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
            break;

        case ePlayerState::PAUSED:
            this->updateStatusText("Paused");
            _ui.playPauseButton->setChecked(false);
            _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            _ui.arucoPushButton->setEnabled(false);
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

void QVideoPlayerWidget::autoStartGStreamer(void)
{
    _ui.playPauseButton->setChecked(true);
    _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-pause"));
    this->startStream(QString::fromStdString(_camURL));
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

void QVideoPlayerWidget::onDisplayDetectedArucos(std::vector<uint16_t> ids_)
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
    _recorderWidget.updateCamURL(_camURL);
    this->hideAngleSelector();
}

void QVideoPlayerWidget::setURLToDefault(void)
{
    _camURL = this->_defaultCamUrl;
    _ui.rtspTextBox->setText(QString::fromStdString(_camURL));
    _recorderWidget.updateCamURL(_camURL);
    this->hideAngleSelector();
}

void QVideoPlayerWidget::updateCamURL()
{
    _camURL = _ui.rtspTextBox->text().toStdString();
    _recorderWidget.updateCamURL(_camURL);
    this->hideAngleSelector();
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
    _recorderWidget.setCameraControlClientManager(client_);
}

void QVideoPlayerWidget::CB_cameraListUpdate(std::vector<std::string> urls_)
{
    _recorderWidget.emitUpdateCameraList(urls_);
}

void QVideoPlayerWidget::CB_srvCameraAvailable(bool available_)
{
    _recorderWidget.CB_srvAvailable(available_);
}

void QVideoPlayerWidget::onCameraAngleSliderChanged(void)
{
    _ui.cameraAngleBox->setValue(_ui.cameraAngleSlider->value());
}

void QVideoPlayerWidget::onCameraAngleBoxChanged(void)
{
    _ui.cameraAngleSlider->setValue(_ui.cameraAngleBox->value());
}

void QVideoPlayerWidget::hideAngleSelector(void)
{
    if (_camURL == Constants::CameraInfo::CAMERA_URL_MAP.at("Main")
        || _camURL == Constants::CameraInfo::CAMERA_URL_MAP.at("Antenna"))
    {
        _ui.cameraAngleSlider->show();
        _ui.cameraAngleBox->show();
        _ui.angleCenterButton->show();
        _ui.panoramaButton->show();
        _ui.actualAngleSlider->show();
        _ui.panoramaDurationBox->show();
    }
    else
    {
        _ui.cameraAngleSlider->hide();
        _ui.cameraAngleBox->hide();
        _ui.angleCenterButton->hide();
        _ui.panoramaButton->hide();
        _ui.actualAngleSlider->hide();
        _ui.panoramaDurationBox->hide();
    }
}

void QVideoPlayerWidget::onCenterAngle(void)
{
    _ui.cameraAngleSlider->setValue(CAMERA_CENTER_ANGLE);
    _ui.cameraAngleBox->setValue(CAMERA_CENTER_ANGLE);
}

void QVideoPlayerWidget::handlePanorama(void)
{
    if (_panoramaWorkerThread.get() != nullptr)
    {
        _panoramaWorkerThread->takePanoramaManager(_client_panoramaManager,
                                                   _camURL,
                                                   _playerIndex,
                                                   _sessionFolderPath,
                                                   _panoramaDuration);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, couldn't access panorama worker");
    }
}

void QVideoPlayerWidget::setPanoramaClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::Panorama>> client_)
{
    if (client_)
    {
        this->_client_panoramaManager = client_;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Couldn't create panorama client");
    }
}

void QVideoPlayerWidget::onPanoramaStarted(uint16_t duration_, uint16_t playerIndex_)
{
    if (_playerIndex == playerIndex_)
    {
        QHelper::QToastNotification::getInstance().notifyFromAnyThread("Panorama started",
                                                                       "Duration: " + std::to_string(duration_ / 1000.0)
                                                                           + " seconds",
                                                                       QHelper::QToastNotification::eNotifType::SUCCESS);
    }
}

void QVideoPlayerWidget::onPanoramaFinished(bool success_, const std::string& status_, uint16_t playerIndex_)
{
    if (_playerIndex == playerIndex_)
    {
        if (success_)
        {
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Panorama finished successfully",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::SUCCESS);
        }
        else
        {
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Panorama failed",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::ERROR);
        }
    }
}

void QVideoPlayerWidget::setPanoramaDuration(void)
{
    _panoramaDuration = _ui.panoramaDurationBox->value() * 1000;
}

void QVideoPlayerWidget::onUpdateActualAngle(const std::string& camURL_, float yaw_)
{
    if (camURL_ == _camURL)
    {
        _ui.actualAngleSlider->setValue(static_cast<int>(yaw_));
    }
}