#include "QVideoPlayerWidget.hpp"
#include "QLogManager.hpp"
#include <QStyle>
#include <QDateTime>
#include <QMessageBox>
#include <QScrollBar>
#include <QRegularExpression>
#include <gst/video/videooverlay.h>

int QVideoPlayerWidget::_instanceCounter = 0;

QVideoPlayerWidget::QVideoPlayerWidget(std::shared_ptr<rclcpp::Node> guiNode_,
                                       std::string url_,
                                       uint16_t tag_,
                                       std::shared_ptr<QPlayerWorker> worker_):
    _node(guiNode_),
    _camURL(url_),
    _tag(tag_),
    _playerWorkerThread(worker_),
    _reconnectTimer(this),
    _frameTimeoutTimer(this),
    _connectionTimeoutTimer(this)
{
    _widgetId = QString("video_player_%1").arg(++_instanceCounter);
    _streamIndex = _instanceCounter - 1;

    _defaultCamUrl = _camURL;
    _ui.setupUi(this);

    this->setupUI();

    _gstreamerThread = new QThread(this);
    _gstreamerWorker = new GStreamerWorker();
    _gstreamerWorker->moveToThread(_gstreamerThread);

    _gstreamerWorker->setTargetId(_widgetId);

    connect(_gstreamerThread, &QThread::finished, _gstreamerWorker, &QObject::deleteLater);
    connect(this, &QVideoPlayerWidget::requestStartStream, _gstreamerWorker, &GStreamerWorker::startPipeline);
    connect(this, &QVideoPlayerWidget::requestStopStream, _gstreamerWorker, &GStreamerWorker::stopPipeline);
    connect(_gstreamerWorker, &GStreamerWorker::pipelineStarted, this, &QVideoPlayerWidget::onPipelineStarted);
    connect(_gstreamerWorker, &GStreamerWorker::errorOccurred, this, &QVideoPlayerWidget::onErrorOccurred);
    connect(_gstreamerWorker, &GStreamerWorker::connectionFailed, this, &QVideoPlayerWidget::onConnectionFailed);
    connect(_gstreamerWorker, &GStreamerWorker::frameReceived, this, &QVideoPlayerWidget::onFrameReceived);

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

    connect(&_frameTimeoutTimer, &QTimer::timeout, this, &QVideoPlayerWidget::onFrameTimeout);
    connect(&_reconnectTimer, &QTimer::timeout, this, &QVideoPlayerWidget::onReconnectTimer);
    connect(&_connectionTimeoutTimer, &QTimer::timeout, this, &QVideoPlayerWidget::onConnectionTimeout);

    connect(&QLogManager::getInstance(), &QLogManager::newLogMessage, this, &QVideoPlayerWidget::onNewLogMessage);

    _ui.rtspTextBox->setText(QString::fromStdString(_camURL));
    _ui.rtspTextBox->setAlignment(Qt::AlignCenter);
    _ui.arucoIdsTextBox->setText("Ids: ");

    this->setPlayerState(ePlayerState::NotConnected);

    _gstreamerThread->start();

    UI_LOG_INFO(GENERAL,
                QString("VideoPlayer Widget initialized for camera: %1").arg(QString::fromStdString(_camURL)),
                _widgetId);
}

QVideoPlayerWidget::~QVideoPlayerWidget()
{
    this->cleanupResources();
}

void QVideoPlayerWidget::cleanupResources(void)
{
    this->stopStream();

    if (_gstreamerThread)
    {
        _gstreamerThread->quit();
        if (!_gstreamerThread->wait(1000))
        {
            _gstreamerThread->terminate();
        }
    }
}

void QVideoPlayerWidget::setupUI(void)
{
    this->storeUIReferences();
    this->connectUISignals();
    this->initializeUIState();

    if (_videoWidget)
    {
        _videoWidget->setStyleSheet("background-color: black;");

        _videoWidget->setVisible(true);
    }

    if (_stackedWidget)
    {
        _stackedWidget->setCurrentIndex(0);
    }

    if (_logDisplay)
    {
        _logDisplay->clear();
        _logDisplay->setStyleSheet("font-family: monospace; color: white; background-color: #222222;");
    }
}

void QVideoPlayerWidget::storeUIReferences(void)
{
    _stackedWidget = _ui.stackedWidget;
    if (!_stackedWidget)
    {
        UI_LOG_ERROR(GENERAL, "Failed to find stackedWidget in UI", _widgetId);
    }

    _videoWidget = _ui.videoWidget;
    if (!_videoWidget)
    {
        UI_LOG_ERROR(GENERAL, "Failed to find videoWidget in UI", _widgetId);
    }

    _videoStack = _ui.videoStack;
    _statusPage = _ui.statusPage;
    _statusLabel = _ui.statusLabel;

    _playPauseButton = _ui.playPauseButton;
    if (!_playPauseButton)
    {
        UI_LOG_ERROR(GENERAL, "Failed to find playPauseButton in UI", _widgetId);
    }

    _toggleViewButton = _ui.toggleViewButton;
    _rtspUrlInput = _ui.rtspTextBox;

    _arucoButton = _ui.arucoPushButton;
    _arucoIdsTextBox = _ui.arucoIdsTextBox;
    _screenshotButton = _ui.ScreenshotButton;
    _recordButton = _ui.startRecordingButton;

    _logDisplay = _ui.logDisplay;
    if (!_logDisplay)
    {
        UI_LOG_WARNING(GENERAL, "Log display not found in UI", _widgetId);
    }

    _debugCheckbox = this->findChild<QCheckBox*>("debugCheckbox");
    _infoCheckbox = this->findChild<QCheckBox*>("infoCheckbox");
    _warningCheckbox = this->findChild<QCheckBox*>("warningCheckbox");
    _errorCheckbox = this->findChild<QCheckBox*>("errorCheckbox");
    _clearButton = this->findChild<QPushButton*>("clearButton");
}

void QVideoPlayerWidget::onNewLogMessage(const QString& message, const QString& target)
{
    if (target == _widgetId && _logDisplay)
    {
        _logDisplay->append(message);

        QScrollBar* scrollBar = _logDisplay->verticalScrollBar();
        if (scrollBar)
        {
            scrollBar->setValue(scrollBar->maximum());
        }
    }
}

void QVideoPlayerWidget::connectUISignals(void)
{
    if (_playPauseButton)
    {
        connect(_playPauseButton,
                &QPushButton::clicked,
                this,
                [this]()
                {
                    if (!_playPauseButton->isChecked())
                    {
                        this->stopStream();
                        _playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
                    }
                    else
                    {
                        this->startStream(QString::fromStdString(_camURL));
                        _playPauseButton->setIcon(QIcon::fromTheme("media-playback-pause"));
                    }
                });
    }

    if (_toggleViewButton)
    {
        connect(_toggleViewButton,
                &QPushButton::clicked,
                this,
                [this]()
                {
                    if (_stackedWidget)
                    {
                        int currentIndex = _stackedWidget->currentIndex();
                        _stackedWidget->setCurrentIndex(currentIndex == 0 ? 1 : 0);
                        UI_LOG_DEBUG_RTSP(QString("Toggled view to index: %1").arg(_stackedWidget->currentIndex()), _widgetId);
                        UI_LOG_INFO_RTSP("Toggled view to logs", _widgetId);
                    }
                });
    }

    if (_rtspUrlInput)
    {
        connect(_rtspUrlInput, &QLineEdit::textChanged, this, &QVideoPlayerWidget::onUrlTextChanged);
    }

    if (_clearButton)
    {
        connect(_clearButton, &QPushButton::clicked, this, &QVideoPlayerWidget::clearLogs);
    }

    QPushButton* backToVideoBtn = this->findChild<QPushButton*>("backToVideoBtn");
    if (backToVideoBtn)
    {
        connect(backToVideoBtn,
                &QPushButton::clicked,
                this,
                [this]()
                {
                    if (_stackedWidget)
                    {
                        _stackedWidget->setCurrentIndex(0);
                    }
                });
    }

    QPushButton* clearLogsBtn = findChild<QPushButton*>("clearLogsBtn");
    if (clearLogsBtn)
    {
        connect(clearLogsBtn, &QPushButton::clicked, this, &QVideoPlayerWidget::clearLogs);
    }

    if (_debugCheckbox)
    {
        connect(_debugCheckbox,
                &QCheckBox::toggled,
                this,
                [this](bool checked)
                {
                    QLogManager::getInstance().setShowDebug(checked, _widgetId);
                });
    }

    if (_infoCheckbox)
    {
        connect(_infoCheckbox,
                &QCheckBox::toggled,
                this,
                [this](bool checked)
                {
                    QLogManager::getInstance().setShowInfo(checked, _widgetId);
                });
    }

    if (_warningCheckbox)
    {
        connect(_warningCheckbox,
                &QCheckBox::toggled,
                this,
                [this](bool checked)
                {
                    QLogManager::getInstance().setShowWarning(checked, _widgetId);
                });
    }

    if (_errorCheckbox)
    {
        connect(_errorCheckbox,
                &QCheckBox::toggled,
                this,
                [this](bool checked)
                {
                    QLogManager::getInstance().setShowError(checked, _widgetId);
                });
    }
}

void QVideoPlayerWidget::initializeUIState(void)
{
    this->updateStatusText("Not Connected");
    this->_controlsVisible = true;

    if (_playPauseButton)
    {
        _playPauseButton->setChecked(false);
        _playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
    }

    if (_arucoButton)
    {
        _arucoButton->setEnabled(false);
    }

    if (_screenshotButton)
    {
        _screenshotButton->setEnabled(false);
    }

    if (_recordButton)
    {
        _recordButton->setEnabled(false);
    }

    if (_debugCheckbox)
    {
        _debugCheckbox->setChecked(false);
    }
    if (_infoCheckbox)
    {
        _infoCheckbox->setChecked(true);
    }
    if (_warningCheckbox)
    {
        _warningCheckbox->setChecked(true);
    }
    if (_errorCheckbox)
    {
        _errorCheckbox->setChecked(true);
    }
}

void QVideoPlayerWidget::startStream(const QString& rtspUrl_)
{
    if (rtspUrl_.isEmpty())
    {
        UI_LOG_WARNING_RTSP("Empty RTSP URL provided", _widgetId);
        return;
    }

    if (!this->validateRtspUrl(rtspUrl_))
    {
        UI_LOG_WARNING_RTSP(QString("Invalid RTSP URL: %1").arg(rtspUrl_), _widgetId);
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

    if (_state != ePlayerState::Reconnecting)
    {
        UI_LOG_INFO_RTSP(QString("Starting stream: %1").arg(rtspUrl_), _widgetId);
        _reconnectAttempts = 0;
    }

    this->setPlayerState(ePlayerState::Connecting);

    emit requestStartStream(rtspUrl_);
}

void QVideoPlayerWidget::stopStream(void)
{
    _connectionTimeoutTimer.stop();

    if (_state == ePlayerState::NotConnected || _state == ePlayerState::Paused)
    {
        return;
    }

    UI_LOG_INFO_RTSP(QString("Stopping stream: %1").arg(QString::fromStdString(_camURL)), _widgetId);

    _frameTimeoutTimer.stop();
    _reconnectTimer.stop();

    if (_arucoButton)
    {
        _arucoButton->setEnabled(false);
    }
    if (_screenshotButton)
    {
        _screenshotButton->setEnabled(false);
    }
    if (_recordButton)
    {
        _recordButton->setEnabled(false);
    }

    emit requestStopStream();

    if (_wasEverConnected)
    {
        this->setPlayerState(ePlayerState::Paused);
    }
    else
    {
        this->setPlayerState(ePlayerState::NotConnected);
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
        case ePlayerState::NotConnected:
            this->updateStatusText("Not Connected");
            if (_playPauseButton)
            {
                _playPauseButton->setChecked(false);
                _playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            }
            break;

        case ePlayerState::Connecting:
            this->updateStatusText("Connecting...");
            if (_playPauseButton)
            {
                _playPauseButton->setChecked(true);
                _playPauseButton->setIcon(QIcon::fromTheme("media-playback-pause"));
            }
            _connectionTimeoutTimer.start(8000);
            break;

        case ePlayerState::Streaming:
            this->updateStatusText("");
            if (_playPauseButton)
            {
                _playPauseButton->setChecked(true);
                _playPauseButton->setIcon(QIcon::fromTheme("media-playback-pause"));
            }
            _wasEverConnected = true;
            if (_arucoButton)
            {
                _arucoButton->setEnabled(true);
            }
            _frameTimeoutTimer.start(2000);
            if (_screenshotButton)
            {
                _screenshotButton->setEnabled(true);
            }
            if (_recordButton)
            {
                _recordButton->setEnabled(true);
            }
            UI_LOG_INFO_RTSP("Stream connected successfully", _widgetId);
            break;

        case ePlayerState::Reconnecting:
            this->updateStatusText(QString("Reconnecting... (%1/%2)").arg(_reconnectAttempts).arg(MAX_RECONNECT_ATTEMPTS));
            if (_playPauseButton)
            {
                _playPauseButton->setChecked(false);
                _playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            }
            if (_arucoButton && _arucoButton->isChecked())
            {
                UI_LOG_INFO_RTSP("Resetting Aruco button due to stream loss", _widgetId);
                _arucoButton->setChecked(false);
                _arucoButton->setProperty("class", "normal");
                _arucoButton->style()->unpolish(_arucoButton);
                _arucoButton->style()->polish(_arucoButton);
                if (_arucoIdsTextBox)
                {
                    _arucoIdsTextBox->setText("Ids: ");
                }
            }
            if (_arucoButton)
            {
                _arucoButton->setEnabled(false);
            }
            if (_screenshotButton)
            {
                _screenshotButton->setEnabled(false);
            }
            if (_recordButton)
            {
                _recordButton->setEnabled(false);
            }
            break;

        case ePlayerState::Paused:
            this->updateStatusText("Paused");
            if (_playPauseButton)
            {
                _playPauseButton->setChecked(false);
                _playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            }
            if (_arucoButton)
            {
                _arucoButton->setEnabled(false);
            }
            if (_screenshotButton)
            {
                _screenshotButton->setEnabled(false);
            }
            if (_recordButton)
            {
                _recordButton->setEnabled(false);
            }
            break;

        case ePlayerState::ConnectionError:
            this->updateStatusText("Connection Error");
            if (_playPauseButton)
            {
                _playPauseButton->setChecked(false);
                _playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            }
            UI_LOG_ERROR_RTSP("Connection error occurred", _widgetId);
            this->tryReconnect();
            break;

        case ePlayerState::ConnectionFailed:
            this->updateStatusText("Connection Failed");
            if (_playPauseButton)
            {
                _playPauseButton->setChecked(false);
                _playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            }
            if (_arucoButton)
            {
                _arucoButton->setEnabled(false);
            }
            if (_screenshotButton)
            {
                _screenshotButton->setEnabled(false);
            }
            if (_recordButton)
            {
                _recordButton->setEnabled(false);
            }
            UI_LOG_ERROR_RTSP("Connection failed permanently", _widgetId);
            break;
    }

    bool wasStreaming = (oldState == ePlayerState::Streaming);
    bool isStreaming = (_state == ePlayerState::Streaming);

    if (wasStreaming != isStreaming)
    {
        this->emitStateChanged();
    }
}

void QVideoPlayerWidget::tryReconnect(void)
{
    if (_state == ePlayerState::ConnectionFailed)
    {
        return;
    }

    _reconnectAttempts++;

    if (_reconnectAttempts <= MAX_RECONNECT_ATTEMPTS)
    {
        UI_LOG_INFO_RTSP(QString("Automatic reconnection attempt %1 of %2").arg(_reconnectAttempts).arg(MAX_RECONNECT_ATTEMPTS),
                         _widgetId);
        this->setPlayerState(ePlayerState::Reconnecting);
        _reconnectTimer.start(3000);
    }
    else
    {
        UI_LOG_ERROR_RTSP("Maximum reconnection attempts reached", _widgetId);
        this->setPlayerState(ePlayerState::ConnectionFailed);
    }
}

void QVideoPlayerWidget::updateStatusText(const QString& text_)
{
    if (!_statusLabel || !_videoStack)
    {
        return;
    }
    _statusLabel->setText(text_);
    _videoStack->setCurrentIndex(text_.isEmpty() ? 0 : 1);
}

bool QVideoPlayerWidget::validateRtspUrl(const QString& url_)
{
    return url_.startsWith("rtsp://") || url_.startsWith("rtspt://") || url_.startsWith("rtsps://");
}

void QVideoPlayerWidget::updateUrlValidationUI(bool isValid_)
{
    if (!_rtspUrlInput)
    {
        return;
    }

    if (isValid_)
    {
        _rtspUrlInput->setStyleSheet("");
        _rtspUrlInput->setToolTip("");
    }
    else
    {
        _rtspUrlInput->setStyleSheet("border: 1px solid red;");
        _rtspUrlInput->setToolTip("Invalid URL format. Expected: rtsp://[username:password@]host[:port]/path");
    }
}

void QVideoPlayerWidget::emitStateChanged(void)
{
    emit streamStateChanged(_state == ePlayerState::Streaming, _streamIndex);
}

void QVideoPlayerWidget::onToggleView(void)
{
    if (_stackedWidget)
    {
        int currentIndex = _stackedWidget->currentIndex();
        int newIndex = (currentIndex == 0) ? 1 : 0;
        _stackedWidget->setCurrentIndex(newIndex);
    }
}

void QVideoPlayerWidget::onUrlTextChanged(const QString& text_)
{
    if (!_rtspUrlInput)
    {
        return;
    }

    if (text_.isEmpty())
    {
        _rtspUrlInput->setStyleSheet("");
        _rtspUrlInput->setToolTip("Enter RTSP URL...");
    }
    else
    {
        bool isValid = this->validateRtspUrl(text_);
        this->updateUrlValidationUI(isValid);

        if (_state == ePlayerState::ConnectionFailed && isValid)
        {
            this->setPlayerState(ePlayerState::NotConnected);
        }
    }
}

void QVideoPlayerWidget::clearLogs(void)
{
    if (_logDisplay)
    {
        _logDisplay->clear();
        UI_LOG_INFO_RTSP("Logs cleared", _widgetId);
    }
}

void QVideoPlayerWidget::toggleLogView(bool show)
{
    if (_stackedWidget)
    {
        _stackedWidget->setCurrentIndex(show ? 1 : 0);
    }
}

void QVideoPlayerWidget::onPipelineStarted(GstElement* pipeline_)
{
    if (!pipeline_)
    {
        UI_LOG_ERROR_RTSP("Pipeline creation failed", _widgetId);
        _connectionTimeoutTimer.stop();
        this->setPlayerState(ePlayerState::ConnectionError);
        return;
    }

    _pipeline = pipeline_;
    GstElement* videoSink = gst_bin_get_by_interface(GST_BIN(_pipeline), GST_TYPE_VIDEO_OVERLAY);

    if (!videoSink)
    {
        UI_LOG_ERROR_RTSP("Failed to find VideoOverlay in pipeline", _widgetId);
        _connectionTimeoutTimer.stop();
        this->setPlayerState(ePlayerState::ConnectionError);
        return;
    }

    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(videoSink), (guintptr)_videoWidget->winId());
    gst_object_unref(videoSink);

    gst_element_set_state(_pipeline, GST_STATE_PLAYING);
    UI_LOG_DEBUG_RTSP("Pipeline state set to PLAYING", _widgetId);

    _frameTimeoutTimer.start(2000);
}

void QVideoPlayerWidget::onErrorOccurred(const QString& error_)
{
    if (_state == ePlayerState::Streaming)
    {
        return;
    }

    if (_state == ePlayerState::Reconnecting)
    {
        UI_LOG_DEBUG_RTSP("Stream error: " + error_, _widgetId);

        if (!_reconnectTimer.isActive())
        {
            _reconnectTimer.start(3000);
        }
    }
    else
    {
        UI_LOG_ERROR_RTSP("Stream error: " + error_, _widgetId);
        this->setPlayerState(ePlayerState::ConnectionError);
    }
}

void QVideoPlayerWidget::onConnectionFailed(void)
{
    _reconnectTimer.stop();
    _connectionTimeoutTimer.stop();
    _reconnectAttempts = 0;
    this->setPlayerState(ePlayerState::ConnectionFailed);
    UI_LOG_ERROR_RTSP("Connection failed permanently", _widgetId);
}

void QVideoPlayerWidget::onFrameReceived(void)
{
    _connectionTimeoutTimer.stop();
    _reconnectTimer.stop();

    if (_state != ePlayerState::Streaming)
    {
        if (_state == ePlayerState::Reconnecting)
        {
            UI_LOG_INFO_RTSP("Reconnection successful, receiving frames...", _widgetId);
            _reconnectAttempts = 0;
        }
        else
        {
            UI_LOG_INFO_RTSP("Receiving frames...", _widgetId);
        }

        this->setPlayerState(ePlayerState::Streaming);

        if (_arucoButton)
        {
            _arucoButton->setEnabled(true);
        }
        if (_screenshotButton)
        {
            _screenshotButton->setEnabled(true);
        }
        if (_recordButton)
        {
            _recordButton->setEnabled(true);
        }
    }
    else
    {
        _frameTimeoutTimer.start(2000);
    }
}

void QVideoPlayerWidget::onFrameTimeout(void)
{
    if (_state == ePlayerState::Streaming)
    {
        UI_LOG_WARNING_RTSP("Frame timeout - no frames received", _widgetId);

        if (_arucoButton)
        {
            _arucoButton->setEnabled(false);
        }
        if (_screenshotButton)
        {
            _screenshotButton->setEnabled(false);
        }
        if (_recordButton)
        {
            _recordButton->setEnabled(false);
        }

        if (_arucoButton && _arucoButton->isChecked())
        {
            UI_LOG_INFO_RTSP("Resetting Aruco button due to frame timeout", _widgetId);
            _arucoButton->setChecked(false);
            _arucoButton->setProperty("class", "normal");
            _arucoButton->style()->unpolish(_arucoButton);
            _arucoButton->style()->polish(_arucoButton);
            if (_arucoIdsTextBox)
            {
                _arucoIdsTextBox->setText("Ids: ");
            }
        }

        this->setPlayerState(ePlayerState::ConnectionError);
    }
}

void QVideoPlayerWidget::onReconnectTimer(void)
{
    if (_state == ePlayerState::Reconnecting)
    {
        if (_rtspUrlInput && !_rtspUrlInput->text().isEmpty())
        {
            this->startStream(_rtspUrlInput->text());
        }
    }
}

void QVideoPlayerWidget::onConnectionTimeout(void)
{
    UI_LOG_ERROR_RTSP("Connection timeout - no response from server", _widgetId);

    if (_arucoButton && _arucoButton->isChecked())
    {
        UI_LOG_INFO_RTSP("Resetting Aruco button due to connection timeout", _widgetId);
        _arucoButton->setChecked(false);
        _arucoButton->setProperty("class", "normal");
        _arucoButton->style()->unpolish(_arucoButton);
        _arucoButton->style()->polish(_arucoButton);
        if (_arucoIdsTextBox)
        {
            _arucoIdsTextBox->setText("Ids: ");
        }
    }

    _reconnectAttempts = 0;
    this->setPlayerState(ePlayerState::ConnectionFailed);
    emit requestStopStream();
}

void QVideoPlayerWidget::handlePlayPauseButton(void)
{
    if (!_playPauseButton)
    {
        return;
    }

    if (!_playPauseButton->isChecked())
    {
        _playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
        this->stopStream();
    }
    else
    {
        _playPauseButton->setIcon(QIcon::fromTheme("media-playback-pause"));
        this->startStream(QString::fromStdString(_camURL));
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
    if (_rtspUrlInput)
    {
        _rtspUrlInput->setText(QString::fromStdString(_camURL));
    }
}

void QVideoPlayerWidget::updateCamURL(void)
{
    if (_rtspUrlInput)
    {
        _camURL = _rtspUrlInput->text().toStdString();
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
        UI_LOG_WARNING(ARUCO_DETECTION, "Error, couldn't access aruco detection manager client", _widgetId);
        if (_arucoButton)
        {
            _arucoButton->setProperty("class", "error");
            _arucoButton->style()->unpolish(_arucoButton);
            _arucoButton->style()->polish(_arucoButton);
        }
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
        UI_LOG_ERROR(ARUCO_DETECTION, "Error, couldn't access Video Player worker", _widgetId);
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
        UI_LOG_ERROR(ARUCO_DETECTION, "Error, couldn't access Video Player worker", _widgetId);
    }
}

void QVideoPlayerWidget::handleArucoDetection(void)
{
    if (!_arucoButton)
    {
        return;
    }

    if (_arucoButton->isChecked())
    {
        UI_LOG_INFO(ARUCO_DETECTION,
                    QString("Starting aruco detection on camera %1").arg(QString::fromStdString(_camURL)),
                    _widgetId);
        this->startDetection();

        if (!_arucoButton->isChecked())
        {
            _arucoButton->setChecked(true);
        }
        _arucoButton->setProperty("class", "success");
        _arucoButton->style()->unpolish(_arucoButton);
        _arucoButton->style()->polish(_arucoButton);
    }
    else
    {
        UI_LOG_INFO(ARUCO_DETECTION,
                    QString("Stopping aruco detection on camera %1").arg(QString::fromStdString(_camURL)),
                    _widgetId);
        this->stopDetection();
        if (_arucoButton->isChecked())
        {
            _arucoButton->setChecked(false);
        }
        _arucoButton->setProperty("class", "normal");
        _arucoButton->style()->unpolish(_arucoButton);
        _arucoButton->style()->polish(_arucoButton);
    }
}

void QVideoPlayerWidget::arucoStillAliveUpdate(bool urlFound_)
{
    if (!_arucoButton)
    {
        return;
    }

    if (!urlFound_ && _arucoButton->isChecked())
    {
        UI_LOG_WARNING(ARUCO_DETECTION,
                       QString("Error, aruco detection on %1 was not found").arg(QString::fromStdString(_camURL)),
                       _widgetId);
        _arucoButton->setProperty("class", "normal");
        _arucoButton->style()->unpolish(_arucoButton);
        _arucoButton->style()->polish(_arucoButton);
    }
}

void QVideoPlayerWidget::displayDetectedArucos(std::vector<uint16_t> ids_)
{
    if (!_arucoIdsTextBox)
    {
        return;
    }

    size_t nbr_ids_detected = ids_.size();

    if (nbr_ids_detected > NBR_IDS_TO_DISPLAY)
    {
        ids_.resize(NBR_IDS_TO_DISPLAY);
    }
    _arucoIdsTextBox->setText("Ids: ");

    for (const auto& id : ids_)
    {
        _arucoIdsTextBox->setText(_arucoIdsTextBox->text() + "  " + QString::number(id));
    }

    if (!ids_.empty())
    {
        UI_LOG_INFO(ARUCO_DETECTION,
                    QString("Detected aruco markers on camera %1: %2")
                        .arg(QString::fromStdString(_camURL))
                        .arg(_arucoIdsTextBox->text().mid(5)),
                    _widgetId);
    }
}

void QVideoPlayerWidget::onDetectionHandledSuccessfully(bool success_, uint16_t tag_)
{
    if (!_arucoButton)
    {
        return;
    }

    if (!success_ && _tag == tag_)
    {
        UI_LOG_ERROR(ARUCO_DETECTION,
                     QString("Error, request made on %1 regarding aruco detection failed").arg(QString::fromStdString(_camURL)),
                     _widgetId);
        _arucoButton->setProperty("class", "error");
        _arucoButton->style()->unpolish(_arucoButton);
        _arucoButton->style()->polish(_arucoButton);
    }
}

void QVideoPlayerWidget::onArucoServerInfoFailed(bool success_)
{
    if (!_arucoButton)
    {
        return;
    }

    if (!success_)
    {
        UI_LOG_DEBUG(ARUCO_DETECTION, "Error, info request to aruco detection manager client failed", _widgetId);
        _arucoButton->setEnabled(false);
    }
    else
    {
        if (_arucoButton->property("class") != "success" && _arucoButton->property("class") != "error")
        {
            _arucoButton->setProperty("class", "normal");
            _arucoButton->setEnabled(true);
            _arucoButton->style()->unpolish(_arucoButton);
            _arucoButton->style()->polish(_arucoButton);
        }
    }
}

void QVideoPlayerWidget::onArucoCameraFailed(bool valid_)
{
    if (!_arucoButton)
    {
        return;
    }

    if (!valid_)
    {
        if (!_arucoButton->isChecked())
        {
            _arucoButton->setChecked(true);
        }
        if (!_arucoButton->isEnabled())
        {
            _arucoButton->setEnabled(true);
        }

        UI_LOG_ERROR(ARUCO_DETECTION,
                     QString("Error, camera at %1 is not accessible").arg(QString::fromStdString(_camURL)),
                     _widgetId);
        _arucoButton->setProperty("class", "error");
        _arucoButton->style()->unpolish(_arucoButton);
        _arucoButton->style()->polish(_arucoButton);
    }

    if (valid_)
    {
        if (!_arucoButton->isChecked())
        {
            _arucoButton->setChecked(true);
        }
        if (!_arucoButton->isEnabled())
        {
            _arucoButton->setEnabled(true);
        }
        _arucoButton->setProperty("class", "success");
        _arucoButton->style()->unpolish(_arucoButton);
        _arucoButton->style()->polish(_arucoButton);
    }
}