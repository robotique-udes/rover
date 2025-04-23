#include "QRtspPlayer/QRtspPlayerWidgetHeader.hpp"
#include "QLoggingMacros.hpp"
#include <QMessageBox>
#include <QScrollBar>
#include <QDateTime>

int RtspPlayerWidget::_instanceCounter = 0;

RtspPlayerWidget::RtspPlayerWidget(QWidget* parent_, const QString& widgetId_):
    QWidget(parent_),
    _workerThread(new QThread(this)),
    _gstreamerWorker(new GStreamerWorker()),
    _reconnectTimer(new QTimer(this)),
    _frameTimeoutTimer(new QTimer(this)),
    _connectionTimeoutTimer(new QTimer(this)),
    _lastStreamUrl(""),
    _lastStreamTime(QDateTime()),
    _pipeline(nullptr),
    _state(PlayerState::NotConnected),
    _reconnectAttempts(0),
    _wasEverConnected(false),
    _controlsVisible(true),
    _arucoDetectionEnabled(false),
    _lastIds{},
    _arucoDetectionClient(nullptr),
    _playerWorkerThread(nullptr),
    _tag(_instanceCounter)
{
    _widgetId = widgetId_.isEmpty() ? QString("rtsp_player_%1").arg(++_instanceCounter) : widgetId_;
    _streamIndex = _instanceCounter - 1;

    for(size_t i=0; i<NBR_IDS_TO_DISPLAY; i++)
    {
        _lastIds[i] = 65535;
    }
    
    // Initialize UI - this now loads the .ui file first
    this->setupUI();

    // Move GStreamer worker to its thread
    _gstreamerWorker->moveToThread(_workerThread);
    connect(_workerThread, &QThread::finished, _gstreamerWorker, &QObject::deleteLater);

    // Connect signals
    this->connectSignals();
    _workerThread->start();
    
    // Initialize state
    this->updateStatusText("Not Connected");
    LOG_INFO_TARGET("RtspPlayer", "RTSP Player Widget initialized", this->_widgetId.toUtf8().constData());
    this->emitStateChanged();
}

RtspPlayerWidget::~RtspPlayerWidget()
{
    this->cleanupResources();
}

void RtspPlayerWidget::connectSignals(void)
{
    // GStreamer worker signals
    connect(this, &RtspPlayerWidget::requestStartStream, _gstreamerWorker, &GStreamerWorker::startPipeline);
    connect(this, &RtspPlayerWidget::requestStopStream, _gstreamerWorker, &GStreamerWorker::stopPipeline);
    connect(_gstreamerWorker, &GStreamerWorker::pipelineStarted, this, &RtspPlayerWidget::onPipelineStarted);
    connect(_gstreamerWorker, &GStreamerWorker::errorOccurred, this, &RtspPlayerWidget::onErrorOccurred);
    connect(_gstreamerWorker, &GStreamerWorker::connectionFailed, this, &RtspPlayerWidget::onConnectionFailed);
    connect(_gstreamerWorker, &GStreamerWorker::frameReceived, this, &RtspPlayerWidget::onFrameReceived);
    
    // URL input validation
    connect(_ui.rtspUrlInput, &QLineEdit::textChanged, this, &RtspPlayerWidget::onUrlTextChanged);
    
    // Timer connections
    _frameTimeoutTimer->setSingleShot(true);
    _reconnectTimer->setSingleShot(true);
    _connectionTimeoutTimer->setSingleShot(true);
    
    connect(_frameTimeoutTimer, &QTimer::timeout, this, &RtspPlayerWidget::onFrameTimeout);
    connect(_reconnectTimer, &QTimer::timeout, this, &RtspPlayerWidget::onReconnectTimer);
    connect(_connectionTimeoutTimer, &QTimer::timeout, this, &RtspPlayerWidget::onConnectionTimeout);
    
    // Aruco detection
    connect(this, &RtspPlayerWidget::arucoCameraFailure, this, &RtspPlayerWidget::onArucoCameraFailed);
}

void RtspPlayerWidget::cleanupResources(void)
{
    this->stopStream();

    if (this->_playerWorkerThread)
    {
        this->_playerWorkerThread->finish();
    }

    if (this->_workerThread)
    {
        this->_workerThread->requestInterruption();
        this->_workerThread->quit();
        if (!this->_workerThread->wait(1000))
        {
            this->_workerThread->terminate();
        }
    }
}

void RtspPlayerWidget::setPlayerState(PlayerState state_)
{
    if (_state == state_)
    {
        return; // Avoid redundant state changes
    }
    
    PlayerState oldState = _state;
    _state = state_;
    
    // Update UI based on state transitions
    switch (_state)
    {
        case PlayerState::NotConnected:
            this->updateStatusText("Not Connected");
            // Update button to play state (unchecked)
            this->_playPauseButton->setChecked(false);
            this->_playPauseButton->setIcon(QIcon(":/icons/play.png"));
            this->_playPauseButton->setToolTip("Play");
            break;
            
        case PlayerState::Connecting:
            this->updateStatusText("Connecting...");
            // Update button to pause state (checked)
            this->_playPauseButton->setChecked(true);
            this->_playPauseButton->setIcon(QIcon(":/icons/stop.png"));
            this->_playPauseButton->setToolTip("Stop");
            this->_connectionTimeoutTimer->start(8000);
            break;
            
        case PlayerState::Streaming:
            this->updateStatusText("");
            // Update button to pause state (checked)
            this->_playPauseButton->setChecked(true);
            this->_playPauseButton->setIcon(QIcon(":/icons/stop.png"));
            this->_playPauseButton->setToolTip("Stop");
            this->_wasEverConnected = true;
            this->_arucoButton->setEnabled(true);
            this->_frameTimeoutTimer->start(2000);
            break;
            
        case PlayerState::Reconnecting:
            this->updateStatusText(QString("Reconnecting... (%1/%2)").arg(this->_reconnectAttempts).arg(MAX_RECONNECT_ATTEMPTS));
            // Update button to play state (unchecked)
            this->_playPauseButton->setChecked(false);
            this->_playPauseButton->setIcon(QIcon(":/icons/play.png"));
            this->_playPauseButton->setToolTip("Play");
            if (this->_arucoButton->isChecked())
            {
                LOG_INFO_TARGET("RtspPlayer", "Automatically stopping Aruco detection due to stream loss", this->_widgetId.toUtf8().constData());
                this->stopArucoDetection();
                this->_arucoButton->setChecked(false);
            }
            this->_arucoButton->setEnabled(false);
            break;
            
        case PlayerState::Paused:
            this->updateStatusText("Paused");
            // Update button to play state (unchecked)
            this->_playPauseButton->setChecked(false);
            this->_playPauseButton->setIcon(QIcon(":/icons/play.png"));
            this->_playPauseButton->setToolTip("Play");
            this->_arucoButton->setEnabled(false);
            break;
            
        case PlayerState::ConnectionError:
            this->updateStatusText("Connection Error");
            // Update button to play state (unchecked)
            this->_playPauseButton->setChecked(false);
            this->_playPauseButton->setIcon(QIcon(":/icons/play.png"));
            this->_playPauseButton->setToolTip("Play");
            this->tryReconnect();
            break;
            
        case PlayerState::ConnectionFailed:
            this->updateStatusText("Connection Failed");
            // Update button to play state (unchecked)
            this->_playPauseButton->setChecked(false);
            this->_playPauseButton->setIcon(QIcon(":/icons/play.png"));
            this->_playPauseButton->setToolTip("Play");
            this->_arucoButton->setEnabled(false);
            break;
    }
    
    // Emit change if streaming state changed
    bool wasStreaming = (oldState == PlayerState::Streaming);
    bool isStreaming = (_state == PlayerState::Streaming);
    
    if (wasStreaming != isStreaming)
    {
        this->emitStateChanged();
    }
}

void RtspPlayerWidget::tryReconnect(void)
{
    if (_state == PlayerState::ConnectionFailed)
    {
        return; // Don't attempt reconnection if permanently failed
    }
    
    _reconnectAttempts++;
    
    if (_reconnectAttempts <= MAX_RECONNECT_ATTEMPTS)
    {
        LOG_INFO_TARGET("RtspPlayer", QString("Automatic reconnection attempt %1 of %2").arg(_reconnectAttempts).arg(MAX_RECONNECT_ATTEMPTS), _widgetId.toUtf8().constData());
        this->setPlayerState(PlayerState::Reconnecting);
        this->_reconnectTimer->start(3000);
    }
    else
    {
        LOG_ERROR_TARGET("RtspPlayer", "Maximum reconnection attempts reached", this->_widgetId.toUtf8().constData());
        this->setPlayerState(PlayerState::ConnectionFailed);
    }
}

void RtspPlayerWidget::emitStateChanged(void)
{
    emit this->streamStateChanged(_state == PlayerState::Streaming, this->_streamIndex);
}

// Signal handlers (former lambdas)
void RtspPlayerWidget::onConnectionFailed(void)
{
    this->_reconnectTimer->stop();
    this->_connectionTimeoutTimer->stop();
    this->_reconnectAttempts = 0;
    this->setPlayerState(PlayerState::ConnectionFailed);
    LOG_ERROR_TARGET("RtspPlayer", "Connection failed permanently", this->_widgetId.toUtf8().constData());
}

void RtspPlayerWidget::onFrameReceived(void)
{
    this->_connectionTimeoutTimer->stop();
    this->_reconnectTimer->stop();
    
    if (_state != PlayerState::Streaming)
    {
        if (_state == PlayerState::Reconnecting)
        {
            LOG_INFO_TARGET("RtspPlayer", "Reconnection successful, receiving frames...", this->_widgetId.toUtf8().constData());
            this->_reconnectAttempts = 0;
        }
        else
        {
            LOG_INFO_TARGET("RtspPlayer", "Receiving frames...", this->_widgetId.toUtf8().constData());
        }
        
        this->setPlayerState(PlayerState::Streaming);

        // Enable control buttons when streaming starts
        this->_arucoButton->setEnabled(true);
        this->_screenshotButton->setEnabled(true);
        this->_recordButton->setEnabled(true);
    }
    else
    {
        // Just reset the frame timeout timer
        this->_frameTimeoutTimer->start(2000);
    }
}

void RtspPlayerWidget::onFrameTimeout(void)
{
    if (_state == PlayerState::Streaming)
    {
        LOG_WARNING_TARGET("RtspPlayer", "Frame timeout - no frames received", this->_widgetId.toUtf8().constData());
        
        // Disable streaming-dependent buttons
        this->_arucoButton->setEnabled(false);
        this->_screenshotButton->setEnabled(false);
        this->_recordButton->setEnabled(false);
        
        // Stop Aruco detection if active
        if (this->_arucoButton->isChecked())
        {
            LOG_INFO_TARGET("RtspPlayer", "Automatically stopping Aruco detection due to stream loss", this->_widgetId.toUtf8().constData());
            this->stopArucoDetection();
            this->_arucoButton->setChecked(false);
        }
        
        // Update state
        this->setPlayerState(PlayerState::ConnectionError);
    }
}

void RtspPlayerWidget::onReconnectTimer(void)
{
    if (_state == PlayerState::Reconnecting)
    {
        if (!this->_ui.rtspUrlInput->text().isEmpty())
        {
            this->startStream(this->_ui.rtspUrlInput->text());
        }
    }
}

void RtspPlayerWidget::onConnectionTimeout(void)
{
    LOG_ERROR_TARGET("RtspPlayer", "Connection timeout - no response from server", this->_widgetId.toUtf8().constData());
    
    if (this->_arucoButton->isChecked())
    {
        LOG_INFO_TARGET("RtspPlayer", "Automatically stopping Aruco detection due to connection timeout", this->_widgetId.toUtf8().constData());
        this->stopArucoDetection();
        this->_arucoButton->setChecked(false);
    }
    
    this->_reconnectAttempts = 0;
    this->setPlayerState(PlayerState::ConnectionFailed);
    emit this->requestStopStream();
}

void RtspPlayerWidget::onUrlTextChanged(const QString& text_)
{
    if (text_.isEmpty())
    {
        this->_ui.rtspUrlInput->setStyleSheet("");
        this->_ui.rtspUrlInput->setToolTip("Enter RTSP URL...");
    }
    else
    {
        bool isValid = this->validateRtspUrl(text_);
        this->updateUrlValidationUI(isValid);
        
        // Reset failure state when URL changes to valid
        if (_state == PlayerState::ConnectionFailed && isValid)
        {
            this->setPlayerState(PlayerState::NotConnected);
        }
    }
}