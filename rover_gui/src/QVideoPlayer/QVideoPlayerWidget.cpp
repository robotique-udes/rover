#include "QVideoPlayerWidget.hpp"
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
    // Generate widget ID
    _widgetId = QString("video_player_%1").arg(++_instanceCounter);
    _streamIndex = _instanceCounter - 1;
    
    _defaultCamUrl = _camURL;
    _ui.setupUi(this);
    
    // Setup UI
    this->setupUI();
    
    // Initialize GStreamer worker thread
    _gstreamerThread = new QThread(this);
    _gstreamerWorker = new GStreamerWorker();
    _gstreamerWorker->moveToThread(_gstreamerThread);
    
    // Connect GStreamer signals
    connect(_gstreamerThread, &QThread::finished, _gstreamerWorker, &QObject::deleteLater);
    connect(this, &QVideoPlayerWidget::requestStartStream, _gstreamerWorker, &GStreamerWorker::startPipeline);
    connect(this, &QVideoPlayerWidget::requestStopStream, _gstreamerWorker, &GStreamerWorker::stopPipeline);
    connect(_gstreamerWorker, &GStreamerWorker::pipelineStarted, this, &QVideoPlayerWidget::onPipelineStarted);
    connect(_gstreamerWorker, &GStreamerWorker::errorOccurred, this, &QVideoPlayerWidget::onErrorOccurred);
    connect(_gstreamerWorker, &GStreamerWorker::connectionFailed, this, &QVideoPlayerWidget::onConnectionFailed);
    connect(_gstreamerWorker, &GStreamerWorker::frameReceived, this, &QVideoPlayerWidget::onFrameReceived);
    
    // Connect Aruco detection signals
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

    // Connect timer signals
    connect(&_frameTimeoutTimer, &QTimer::timeout, this, &QVideoPlayerWidget::onFrameTimeout);
    connect(&_reconnectTimer, &QTimer::timeout, this, &QVideoPlayerWidget::onReconnectTimer);
    connect(&_connectionTimeoutTimer, &QTimer::timeout, this, &QVideoPlayerWidget::onConnectionTimeout);
    
    // Initialize UI state
    _ui.rtspTextBox->setText(QString::fromStdString(_camURL));
    _ui.rtspTextBox->setAlignment(Qt::AlignCenter);
    _ui.arucoIdsTextBox->setText("Ids: ");
    
    // Initialize state
    this->setPlayerState(PlayerState::NotConnected);
    
    // Start GStreamer thread
    _gstreamerThread->start();
    
    // Log initialization
    RCLCPP_INFO(_node->get_logger(), "VideoPlayer Widget initialized for camera: %s", _camURL.c_str());
}

QVideoPlayerWidget::~QVideoPlayerWidget()
{
    this->cleanupResources();
}

void QVideoPlayerWidget::cleanupResources()
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
    
    // Make sure we initialize the video area properly
    if (_videoWidget) {
        // Set background color to black
        _videoWidget->setStyleSheet("background-color: black;");
        // Ensure it's visible
        _videoWidget->setVisible(true);
    }
    
    // Make sure the stacked widget starts on the correct page
    if (_stackedWidget) {
        _stackedWidget->setCurrentIndex(0); // Video page
    }
    
    // Make sure the logDisplay is properly initialized
    if (_logDisplay) {
        _logDisplay->clear();
        _logDisplay->append("Log display initialized for camera: " + QString::fromStdString(_camURL));
    }
}

void QVideoPlayerWidget::storeUIReferences(void)
{
    // Store widget references with better error handling
    _stackedWidget = _ui.stackedWidget;
    if (!_stackedWidget) {
        RCLCPP_ERROR(_node->get_logger(), "Failed to find stackedWidget in UI");
    }
    
    _videoWidget = _ui.videoWidget;
    if (!_videoWidget) {
        RCLCPP_ERROR(_node->get_logger(), "Failed to find videoWidget in UI");
    }
    
    _videoStack = _ui.videoStack;
    _statusPage = _ui.statusPage;
    _statusLabel = _ui.statusLabel;
    
    _playPauseButton = _ui.playPauseButton;
    if (!_playPauseButton) {
        RCLCPP_ERROR(_node->get_logger(), "Failed to find playPauseButton in UI");
    }
    
    _toggleViewButton = _ui.toggleViewButton;
    _rtspUrlInput = _ui.rtspTextBox;
    
    _arucoButton = _ui.arucoPushButton;
    _arucoIdsTextBox = _ui.arucoIdsTextBox;
    _screenshotButton = _ui.ScreenshotButton;
    _recordButton = _ui.startRecordingButton;
    
    _logDisplay = _ui.logDisplay;
    if (!_logDisplay) {
        RCLCPP_WARN(_node->get_logger(), "Log display not found in UI");
    }
    
    // Try to find other log controls
    _debugCheckbox = this->findChild<QCheckBox*>("debugCheckbox");
    _infoCheckbox = this->findChild<QCheckBox*>("infoCheckbox");
    _warningCheckbox = this->findChild<QCheckBox*>("warningCheckbox");
    _errorCheckbox = this->findChild<QCheckBox*>("errorCheckbox");
    _clearButton = this->findChild<QPushButton*>("clearButton");
}

void QVideoPlayerWidget::connectUISignals(void)
{
    // Fix play/pause button connection
    if (_playPauseButton) {
        connect(_playPauseButton, &QPushButton::clicked, this, [this]() {
            if (!_playPauseButton->isChecked()) {
                // Button is now unchecked - was checked before click
                this->stopStream();
                _playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            } else {
                // Button is now checked - was unchecked before click
                this->startStream(QString::fromStdString(_camURL));
                _playPauseButton->setIcon(QIcon::fromTheme("media-playback-pause"));
            }
        });
    }
    
    // Fix toggle view button to properly switch between video and log view
    if (_toggleViewButton) {
        connect(_toggleViewButton, &QPushButton::clicked, this, [this]() {
            if (_stackedWidget) {
                int currentIndex = _stackedWidget->currentIndex();
                _stackedWidget->setCurrentIndex(currentIndex == 0 ? 1 : 0);
                RCLCPP_DEBUG(_node->get_logger(), "Toggled view to index: %d", _stackedWidget->currentIndex());
            }
        });
    }
    
    // Connect URL text changed
    if (_rtspUrlInput) {
        connect(_rtspUrlInput, &QLineEdit::textChanged, this, &QVideoPlayerWidget::onUrlTextChanged);
    }
    
    // Connect clear log button if it exists
    if (_clearButton) {
        connect(_clearButton, &QPushButton::clicked, this, &QVideoPlayerWidget::clearLogs);
    }
    
    // Add back button to log view if it exists
    QPushButton* backToVideoBtn = this->findChild<QPushButton*>("backToVideoBtn");
    if (backToVideoBtn) {
        connect(backToVideoBtn, &QPushButton::clicked, this, [this]() {
            if (_stackedWidget) {
                _stackedWidget->setCurrentIndex(0); // Switch to video view
            }
        });
    }
}

void QVideoPlayerWidget::initializeUIState(void)
{
    this->updateStatusText("Not Connected");
    this->_controlsVisible = true;
    
    // Initialize buttons - make sure unchecked initially
    if (_playPauseButton) {
        _playPauseButton->setChecked(false);
        _playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
    }
    
    if (_arucoButton) {
        _arucoButton->setEnabled(false);
    }
    
    if (_screenshotButton) {
        _screenshotButton->setEnabled(false);
    }
    
    if (_recordButton) {
        _recordButton->setEnabled(false);
    }
    
    // Initialize checkboxes if they exist
    if (_debugCheckbox) _debugCheckbox->setChecked(false);
    if (_infoCheckbox) _infoCheckbox->setChecked(true);
    if (_warningCheckbox) _warningCheckbox->setChecked(true);
    if (_errorCheckbox) _errorCheckbox->setChecked(true);
}

// Stream handling methods
void QVideoPlayerWidget::startStream(const QString& rtspUrl_)
{
    if (rtspUrl_.isEmpty())
    {
        RCLCPP_WARN(_node->get_logger(), "Empty RTSP URL provided");
        return;
    }

    if (!this->validateRtspUrl(rtspUrl_))
    {
        RCLCPP_WARN(_node->get_logger(), "Invalid RTSP URL: %s", rtspUrl_.toStdString().c_str());
        QMessageBox::warning(this, "Invalid RTSP URL",
                           "The URL format is invalid. Please enter a valid RTSP URL.\n\n"
                           "Format: rtsp://[username:password@]host[:port]/path");
        return;
    }
    
    QDateTime currentTime = QDateTime::currentDateTime();
    if (rtspUrl_ == QString::fromStdString(_camURL) && _lastStreamTime.isValid() && 
        _lastStreamTime.msecsTo(currentTime) < 500)
    {
        return;
    }
    
    _lastStreamTime = currentTime;
    _camURL = rtspUrl_.toStdString();

    if (_state != PlayerState::Reconnecting)
    {
        RCLCPP_INFO(_node->get_logger(), "Starting stream: %s", rtspUrl_.toStdString().c_str());
        _reconnectAttempts = 0;
    }

    this->setPlayerState(PlayerState::Connecting);
    
    // Request to start the pipeline
    emit requestStartStream(rtspUrl_);
}

void QVideoPlayerWidget::stopStream(void)
{
    _connectionTimeoutTimer.stop();
    
    if (_state == PlayerState::NotConnected || _state == PlayerState::Paused)
    {
        return;
    }

    RCLCPP_INFO(_node->get_logger(), "Stopping stream: %s", _camURL.c_str());
    
    _frameTimeoutTimer.stop();
    _reconnectTimer.stop();
    
    if (_arucoButton) _arucoButton->setEnabled(false);
    if (_screenshotButton) _screenshotButton->setEnabled(false);
    if (_recordButton) _recordButton->setEnabled(false);
    
    // Request to stop the pipeline
    emit requestStopStream();
    
    if (_wasEverConnected)
    {
        this->setPlayerState(PlayerState::Paused);
    }
    else
    {
        this->setPlayerState(PlayerState::NotConnected);
    }
}

void QVideoPlayerWidget::setPlayerState(PlayerState state_)
{
    if (_state == state_)
    {
        return; 
    }
    
    PlayerState oldState = _state;
    _state = state_;
    
    switch (_state)
    {
        case PlayerState::NotConnected:
            this->updateStatusText("Not Connected");
            if (_playPauseButton) {
                _playPauseButton->setChecked(false);
                _playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            }
            break;
            
        case PlayerState::Connecting:
            this->updateStatusText("Connecting...");
            if (_playPauseButton) {
                _playPauseButton->setChecked(true);
                _playPauseButton->setIcon(QIcon::fromTheme("media-playback-pause"));
            }
            _connectionTimeoutTimer.start(8000);
            break;
            
        case PlayerState::Streaming:
            this->updateStatusText("");
            if (_playPauseButton) {
                _playPauseButton->setChecked(true);
                _playPauseButton->setIcon(QIcon::fromTheme("media-playback-pause"));
            }
            _wasEverConnected = true;
            if (_arucoButton) _arucoButton->setEnabled(true);
            _frameTimeoutTimer.start(2000);
            if (_screenshotButton) _screenshotButton->setEnabled(true);
            if (_recordButton) _recordButton->setEnabled(true);
            break;
            
        case PlayerState::Reconnecting:
            this->updateStatusText(QString("Reconnecting... (%1/%2)").arg(_reconnectAttempts).arg(MAX_RECONNECT_ATTEMPTS));
            if (_playPauseButton) {
                _playPauseButton->setChecked(false);
                _playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            }
            if (_arucoButton && _arucoButton->isChecked())
            {
                RCLCPP_INFO(_node->get_logger(), "Resetting Aruco button due to stream loss");
                _arucoButton->setChecked(false);
                _arucoButton->setProperty("class", "normal");
                _arucoButton->style()->unpolish(_arucoButton);
                _arucoButton->style()->polish(_arucoButton);
                if (_arucoIdsTextBox) _arucoIdsTextBox->setText("Ids: ");
            }
            if (_arucoButton) _arucoButton->setEnabled(false);
            if (_screenshotButton) _screenshotButton->setEnabled(false);
            if (_recordButton) _recordButton->setEnabled(false);
            break;
            
        case PlayerState::Paused:
            this->updateStatusText("Paused");
            if (_playPauseButton) {
                _playPauseButton->setChecked(false);
                _playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            }
            if (_arucoButton) _arucoButton->setEnabled(false);
            if (_screenshotButton) _screenshotButton->setEnabled(false);
            if (_recordButton) _recordButton->setEnabled(false);
            break;
            
        case PlayerState::ConnectionError:
            this->updateStatusText("Connection Error");
            if (_playPauseButton) {
                _playPauseButton->setChecked(false);
                _playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            }
            this->tryReconnect();
            break;
            
        case PlayerState::ConnectionFailed:
            this->updateStatusText("Connection Failed");
            if (_playPauseButton) {
                _playPauseButton->setChecked(false);
                _playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            }
            if (_arucoButton) _arucoButton->setEnabled(false);
            if (_screenshotButton) _screenshotButton->setEnabled(false);
            if (_recordButton) _recordButton->setEnabled(false);
            break;
    }
    
    bool wasStreaming = (oldState == PlayerState::Streaming);
    bool isStreaming = (_state == PlayerState::Streaming);
    
    if (wasStreaming != isStreaming)
    {
        this->emitStateChanged();
    }
}

void QVideoPlayerWidget::tryReconnect(void)
{
    if (_state == PlayerState::ConnectionFailed)
    {
        return; 
    }
    
    _reconnectAttempts++;
    
    if (_reconnectAttempts <= MAX_RECONNECT_ATTEMPTS)
    {
        RCLCPP_INFO(_node->get_logger(), "Automatic reconnection attempt %d of %d", 
                   _reconnectAttempts, MAX_RECONNECT_ATTEMPTS);
        this->setPlayerState(PlayerState::Reconnecting);
        _reconnectTimer.start(3000);
    }
    else
    {
        RCLCPP_ERROR(_node->get_logger(), "Maximum reconnection attempts reached");
        this->setPlayerState(PlayerState::ConnectionFailed);
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
    // Simple validation - can be enhanced
    return url_.startsWith("rtsp://") || url_.startsWith("rtspt://") || url_.startsWith("rtsps://");
}

void QVideoPlayerWidget::updateUrlValidationUI(bool isValid_)
{
    if (!_rtspUrlInput) return;
    
    if (isValid_) {
        _rtspUrlInput->setStyleSheet("");
        _rtspUrlInput->setToolTip("");
    } else {
        _rtspUrlInput->setStyleSheet("border: 1px solid red;");
        _rtspUrlInput->setToolTip("Invalid URL format. Expected: rtsp://[username:password@]host[:port]/path");
    }
}

void QVideoPlayerWidget::emitStateChanged(void)
{
    emit streamStateChanged(_state == PlayerState::Streaming, _streamIndex);
}

void QVideoPlayerWidget::onToggleView(void)
{
    if (_stackedWidget) {
        int currentIndex = _stackedWidget->currentIndex();
        int newIndex = (currentIndex == 0) ? 1 : 0;
        _stackedWidget->setCurrentIndex(newIndex);
    }
}

void QVideoPlayerWidget::onUrlTextChanged(const QString& text_)
{
    if (!_rtspUrlInput) return;
    
    if (text_.isEmpty())
    {
        _rtspUrlInput->setStyleSheet("");
        _rtspUrlInput->setToolTip("Enter RTSP URL...");
    }
    else
    {
        bool isValid = this->validateRtspUrl(text_);
        this->updateUrlValidationUI(isValid);
        
        if (_state == PlayerState::ConnectionFailed && isValid)
        {
            this->setPlayerState(PlayerState::NotConnected);
        }
    }
}

// Log-related methods
void QVideoPlayerWidget::onNewLogMessage(const QString& message, const QString& target)
{
    if (target == _widgetId && _logDisplay)
    {
        _logDisplay->append(message);
        
        // Auto-scroll to bottom
        QScrollBar* scrollBar = _logDisplay->verticalScrollBar();
        if (scrollBar) {
            scrollBar->setValue(scrollBar->maximum());
        }
    }
}

void QVideoPlayerWidget::clearLogs()
{
    if (_logDisplay) {
        _logDisplay->clear();
        _logDisplay->append("Logs cleared for Video Player " + _widgetId);
    }
}

void QVideoPlayerWidget::toggleLogView(bool show)
{
    if (_stackedWidget) {
        _stackedWidget->setCurrentIndex(show ? 1 : 0);
    }
}

// GStreamer-related slots
void QVideoPlayerWidget::onPipelineStarted(GstElement* pipeline_)
{
    if (!pipeline_)
    {
        RCLCPP_ERROR(_node->get_logger(), "Pipeline creation failed");
        _connectionTimeoutTimer.stop();
        this->setPlayerState(PlayerState::ConnectionError);
        return;
    }

    _pipeline = pipeline_;
    GstElement* videoSink = gst_bin_get_by_interface(GST_BIN(_pipeline), GST_TYPE_VIDEO_OVERLAY);
    
    if (!videoSink)
    {
        RCLCPP_ERROR(_node->get_logger(), "Failed to find VideoOverlay in pipeline");
        _connectionTimeoutTimer.stop();
        this->setPlayerState(PlayerState::ConnectionError);
        return;
    }

    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(videoSink), (guintptr)_videoWidget->winId());
    gst_object_unref(videoSink);
    
    gst_element_set_state(_pipeline, GST_STATE_PLAYING);
    RCLCPP_DEBUG(_node->get_logger(), "Pipeline state set to PLAYING");
    
    _frameTimeoutTimer.start(2000);
}

void QVideoPlayerWidget::onErrorOccurred(const QString& error_)
{
    if (_state == PlayerState::Streaming)
    {
        return;
    }
    
    if (_state == PlayerState::Reconnecting)
    {
        RCLCPP_DEBUG(_node->get_logger(), "Stream error: %s", error_.toStdString().c_str());
        
        if (!_reconnectTimer.isActive())
        {
            _reconnectTimer.start(3000);
        }
    }
    else
    {
        RCLCPP_ERROR(_node->get_logger(), "Stream error: %s", error_.toStdString().c_str());
        this->setPlayerState(PlayerState::ConnectionError);
    }
}

void QVideoPlayerWidget::onConnectionFailed(void)
{
    _reconnectTimer.stop();
    _connectionTimeoutTimer.stop();
    _reconnectAttempts = 0;
    this->setPlayerState(PlayerState::ConnectionFailed);
    RCLCPP_ERROR(_node->get_logger(), "Connection failed permanently");
}

void QVideoPlayerWidget::onFrameReceived(void)
{
    _connectionTimeoutTimer.stop();
    _reconnectTimer.stop();
    
    if (_state != PlayerState::Streaming)
    {
        if (_state == PlayerState::Reconnecting)
        {
            RCLCPP_INFO(_node->get_logger(), "Reconnection successful, receiving frames...");
            _reconnectAttempts = 0;
        }
        else
        {
            RCLCPP_INFO(_node->get_logger(), "Receiving frames...");
        }
        
        this->setPlayerState(PlayerState::Streaming);

        if (_arucoButton) _arucoButton->setEnabled(true);
        if (_screenshotButton) _screenshotButton->setEnabled(true);
        if (_recordButton) _recordButton->setEnabled(true);
    }
    else
    {
        _frameTimeoutTimer.start(2000);
    }
}

void QVideoPlayerWidget::onFrameTimeout(void)
{
    if (_state == PlayerState::Streaming)
    {
        RCLCPP_WARN(_node->get_logger(), "Frame timeout - no frames received");

        if (_arucoButton) _arucoButton->setEnabled(false);
        if (_screenshotButton) _screenshotButton->setEnabled(false);
        if (_recordButton) _recordButton->setEnabled(false);
        
        if (_arucoButton && _arucoButton->isChecked())
        {
            RCLCPP_INFO(_node->get_logger(), "Resetting Aruco button due to frame timeout");
            _arucoButton->setChecked(false);
            _arucoButton->setProperty("class", "normal");
            _arucoButton->style()->unpolish(_arucoButton);
            _arucoButton->style()->polish(_arucoButton);
            if (_arucoIdsTextBox) _arucoIdsTextBox->setText("Ids: ");
        }
        
        this->setPlayerState(PlayerState::ConnectionError);
    }
}

void QVideoPlayerWidget::onReconnectTimer(void)
{
    if (_state == PlayerState::Reconnecting)
    {
        if (_rtspUrlInput && !_rtspUrlInput->text().isEmpty())
        {
            this->startStream(_rtspUrlInput->text());
        }
    }
}

void QVideoPlayerWidget::onConnectionTimeout(void)
{
    RCLCPP_ERROR(_node->get_logger(), "Connection timeout - no response from server");
    
    if (_arucoButton && _arucoButton->isChecked())
    {
        RCLCPP_INFO(_node->get_logger(), "Resetting Aruco button due to connection timeout");
        _arucoButton->setChecked(false);
        _arucoButton->setProperty("class", "normal");
        _arucoButton->style()->unpolish(_arucoButton);
        _arucoButton->style()->polish(_arucoButton);
        if (_arucoIdsTextBox) _arucoIdsTextBox->setText("Ids: ");
    }
    
    _reconnectAttempts = 0;
    this->setPlayerState(PlayerState::ConnectionFailed);
    emit requestStopStream();
}

// Aruco detection methods - kept unchanged
void QVideoPlayerWidget::handlePlayPauseButton(void)
{
    if (!_playPauseButton) return;
    
    if (!_playPauseButton->isChecked()) {
        // Button is unchecked - stop the stream
        _playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
        this->stopStream();
    } else {
        // Button is checked - start the stream
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
    if (_rtspUrlInput) {
        _rtspUrlInput->setText(QString::fromStdString(_camURL));
    }
}

void QVideoPlayerWidget::updateCamURL(void)
{
    if (_rtspUrlInput) {
        _camURL = _rtspUrlInput->text().toStdString();
    }
}

// Aruco detection methods
void QVideoPlayerWidget::setArucoClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_)
{
    if (client_)
    {
        this->_client_arucoManager = client_;
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error, couldn't access aruco detection manager client");
        if (_arucoButton) {
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
    if (!_arucoButton) return;
    
    if (_arucoButton->isChecked())
    {
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
    if (!_arucoButton) return;
    
    if (!urlFound_ && _arucoButton->isChecked())
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error, aruco detection on %s was not found", _camURL.c_str());
        _arucoButton->setProperty("class", "normal");
        _arucoButton->style()->unpolish(_arucoButton);
        _arucoButton->style()->polish(_arucoButton);
    }
}

void QVideoPlayerWidget::displayDetectedArucos(std::vector<uint16_t> ids_)
{
    if (!_arucoIdsTextBox) return;
    
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
}

void QVideoPlayerWidget::onDetectionHandledSuccessfully(bool success_, uint16_t tag_)
{
    if (!_arucoButton) return;
    
    if (!success_ && _tag == tag_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, request made on %s regarding aruco detection failed", _camURL.c_str());
        _arucoButton->setProperty("class", "error");
        _arucoButton->style()->unpolish(_arucoButton);
        _arucoButton->style()->polish(_arucoButton);
    }
}

void QVideoPlayerWidget::onArucoServerInfoFailed(bool success_)
{
    if (!_arucoButton) return;
    
    if (!success_)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Error, info request to aruco detection manager client failed");
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
    if (!_arucoButton) return;
    
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

        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, camera at %s is not accessible", _camURL.c_str());
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