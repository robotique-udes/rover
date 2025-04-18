#include "QRtspPlayerWidget.hpp"
#include "QLoggingMacros.hpp"
#include <QMessageBox>
#include <QScrollBar>
#include <gst/video/videooverlay.h>
#include <QRegularExpression>
#include <QEvent>
#include <QMessageBox>
#include <QStyle>
#include <QToolButton>
#include <QDateTime>

int RtspPlayerWidget::instanceCounter = 0;

RtspPlayerWidget::RtspPlayerWidget(QWidget* parent, const QString& widgetId):
    QWidget(parent),
    workerThread(new QThread(this)),
    gstreamerWorker(new GStreamerWorker()),
    reconnectTimer(new QTimer(this)),
    frameTimeoutTimer(new QTimer(this)),
    connectionTimeoutTimer(new QTimer(this)),
    _lastStreamUrl(""),
    _lastStreamTime(QDateTime()),
    pipeline(nullptr),
    receivingFrames(false),
    inReconnectionMode(false),
    reconnectAttempts(0),
    wasEverConnected(false),
    connectionFailed(false),
    _controlsVisible(true),
    _arucoDetectionEnabled(false),
    _lastIds{},
    _arucoDetectionClient(nullptr),
    _playerWorkerThread(nullptr),
    _tag(instanceCounter)
{
    _widgetId = widgetId.isEmpty() ? QString("rtsp_player_%1").arg(++instanceCounter) : widgetId;
    _streamIndex = instanceCounter - 1;

    for(size_t i=0; i<NBR_IDS_TO_DISPLAY; i++) {
        _lastIds[i] = 65535;
    }
    
    // Setup UI components and signal connections
    setupUI();

    gstreamerWorker->moveToThread(workerThread);
    connect(workerThread, &QThread::finished, gstreamerWorker, &QObject::deleteLater);

    connect(this, &RtspPlayerWidget::requestStartStream, gstreamerWorker, &GStreamerWorker::startPipeline);
    connect(this, &RtspPlayerWidget::requestStopStream, gstreamerWorker, &GStreamerWorker::stopPipeline);
    connect(gstreamerWorker, &GStreamerWorker::pipelineStarted, this, &RtspPlayerWidget::onPipelineStarted);
    connect(gstreamerWorker, &GStreamerWorker::errorOccurred, this, &RtspPlayerWidget::onErrorOccurred);
    
    // Validate RTSP URL input changes
    connect(ui.rtspUrlInput, &QLineEdit::textChanged, this, [this](const QString& text) {
        if (text.isEmpty()) {
            ui.rtspUrlInput->setStyleSheet("");
            ui.rtspUrlInput->setToolTip("Enter RTSP URL...");
        } else {
            bool isValid = validateRtspUrl(text);
            updateUrlValidationUI(isValid);
            if (connectionFailed && isValid) {
                connectionFailed = false;
            }
        }
    });
    
    // Handle connection failures from GStreamerWorker
    connect(gstreamerWorker, &GStreamerWorker::connectionFailed, this, [this]() {
        inReconnectionMode = false;
        reconnectAttempts = 0;
        reconnectTimer->stop();
        connectionTimeoutTimer->stop();
        connectionFailed = true;
        updateStatusText("Connection Failed");
        _playPauseButton->setPlaying(false);
        LOG_ERROR_TARGET("RtspPlayer", "Connection failed permanently", _widgetId.toUtf8().constData());
    });

    connect(gstreamerWorker,
            &GStreamerWorker::frameReceived,
            this,
            [this]()
            {
                connectionTimeoutTimer->stop();
                
                if (!receivingFrames)
                {
                    if (inReconnectionMode)
                    {
                        LOG_INFO_TARGET("RtspPlayer", "Reconnection successful, receiving frames...", _widgetId.toUtf8().constData());
                        inReconnectionMode = false;
                        reconnectAttempts = 0;
                    }
                    else
                    {
                        LOG_INFO_TARGET("RtspPlayer", "Receiving frames...", _widgetId.toUtf8().constData());
                    }
                    receivingFrames = true;
                    wasEverConnected = true;
                    connectionFailed = false;
                    _playPauseButton->setPlaying(true);
                    emitStateChanged();
                    updateStatusText("");
                    _arucoButton->setEnabled(true);
                    _screenshotButton->setEnabled(true);
                    _recordButton->setEnabled(true);
                }
                reconnectTimer->stop();
                frameTimeoutTimer->start(2000);
            });

    frameTimeoutTimer->setSingleShot(true);
    connect(frameTimeoutTimer,
            &QTimer::timeout,
            this,
            [this]()
            {
                if (receivingFrames)
                {
                    receivingFrames = false;
                    LOG_WARNING_TARGET("RtspPlayer", "Frame timeout - no frames received", _widgetId.toUtf8().constData());
                    _playPauseButton->setPlaying(false);
                    if (_arucoButton->isChecked()) {
                        LOG_INFO_TARGET("RtspPlayer", "Automatically stopping Aruco detection due to stream loss", _widgetId.toUtf8().constData());
                        stopArucoDetection();
                        _arucoButton->setChecked(false);
                    }
                    inReconnectionMode = true;
                    updateStatusText("Connection Lost");
                    reconnectTimer->start(3000);
                    _arucoButton->setEnabled(false);
                    _screenshotButton->setEnabled(false);
                    _recordButton->setEnabled(false);
                    emitStateChanged();
                }
            });

    reconnectTimer->setSingleShot(true);
    connect(reconnectTimer,
            &QTimer::timeout,
            this,
            [this]()
            {
                if (!receivingFrames)
                {
                    reconnectAttempts++;
                    
                    if (reconnectAttempts <= maxReconnectAttempts)
                    {
                        updateStatusText(QString("Reconnecting... (%1/%2)").arg(reconnectAttempts).arg(maxReconnectAttempts));
                        LOG_INFO_TARGET("RtspPlayer", QString("Automatic reconnection attempt %1 of %2").arg(reconnectAttempts).arg(maxReconnectAttempts), _widgetId.toUtf8().constData());
                        inReconnectionMode = true;

                        if (!ui.rtspUrlInput->text().isEmpty())
                        {
                            startStream(ui.rtspUrlInput->text());
                        }
                    }
                    else
                    {
                        inReconnectionMode = false;
                        connectionFailed = true;
                        updateStatusText("Connection Failed");
                        _playPauseButton->setPlaying(false);
                        LOG_ERROR_TARGET("RtspPlayer", "Maximum reconnection attempts reached", _widgetId.toUtf8().constData());
                    }
                }
            });
            
    connectionTimeoutTimer->setSingleShot(true);
    connect(connectionTimeoutTimer, &QTimer::timeout, this, [this]() {
        LOG_ERROR_TARGET("RtspPlayer", "Connection timeout - no response from server", _widgetId.toUtf8().constData());
        if (_arucoButton->isChecked()) {
            LOG_INFO_TARGET("RtspPlayer", "Automatically stopping Aruco detection due to connection timeout", _widgetId.toUtf8().constData());
            stopArucoDetection();
            _arucoButton->setChecked(false);
        }
        inReconnectionMode = false;
        reconnectAttempts = 0;
        connectionFailed = true;
        updateStatusText("Connection Timeout");
        _playPauseButton->setPlaying(false);
        emit requestStopStream();
    });

    connect(this, &RtspPlayerWidget::arucoCameraFailure, this, &RtspPlayerWidget::onArucoCameraFailed);
    
    workerThread->start();
    
    updateStatusText("Not Connected");
    
    LOG_INFO_TARGET("RtspPlayer", "RTSP Player Widget initialized", _widgetId.toUtf8().constData());
    emitStateChanged();
}

RtspPlayerWidget::~RtspPlayerWidget()
{
    stopStream();

    if (_playerWorkerThread) {
        _playerWorkerThread->finish();
    }

    if (workerThread) {
        workerThread->requestInterruption();
        workerThread->quit();
        if (!workerThread->wait(1000)) {
            workerThread->terminate();
        }
    }
}

void RtspPlayerWidget::setupEnhancedControls()
{
    QHBoxLayout* topLayout = ui.topLayout;
    if (!topLayout) {
        LOG_ERROR_TARGET("RtspPlayer", "Could not find top layout to add enhanced controls", _widgetId.toUtf8().constData());
        return;
    }

    _arucoIdsTextBox = new QLineEdit(this);
    _arucoIdsTextBox->setReadOnly(true);
    _arucoIdsTextBox->setText("Ids: ");
    _arucoIdsTextBox->setAlignment(Qt::AlignCenter);
    _arucoIdsTextBox->setMinimumWidth(120);

    _arucoButton = new QPushButton(this);
    _arucoButton->setText("Aruco");
    _arucoButton->setCheckable(true);
    _arucoButton->setEnabled(false);
    _arucoButton->setToolTip("Enable Aruco marker detection");
    _arucoButton->setObjectName("arucoPushButton");
    connect(_arucoButton, &QPushButton::clicked, this, &RtspPlayerWidget::onArucoButtonClicked);

    _screenshotButton = new QPushButton(this);
    _screenshotButton->setText("Screenshot");
    _screenshotButton->setEnabled(false);
    _screenshotButton->setToolTip("Take a screenshot");

    _recordButton = new QPushButton(this);
    _recordButton->setText("Record");
    _recordButton->setCheckable(true);
    _recordButton->setEnabled(false);
    _recordButton->setToolTip("Start/stop recording");

    topLayout->insertWidget(0, _arucoButton);
    topLayout->insertWidget(1, _arucoIdsTextBox);
    topLayout->insertWidget(2, _screenshotButton);
    topLayout->insertWidget(3, _recordButton);
    topLayout->insertStretch(4);
}

void RtspPlayerWidget::setupUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(0);
    
    _stackedWidget = new QStackedWidget(this);
    
    _videoWidget = new QWidget();
    ui.setupUi(_videoWidget);
    
    if (ui.statusIndicator) {
        ui.statusIndicator->hide();
        ui.statusIndicator->setMaximumSize(0, 0);
    }
    
    QWidget* videoContainer = ui.videoWidget->parentWidget();
    QLayout* originalLayout = nullptr;
    
    if (videoContainer) {
        originalLayout = videoContainer->layout();
        originalLayout->removeWidget(ui.videoWidget);
        _videoStack = new QStackedWidget(videoContainer);
        _videoStack->addWidget(ui.videoWidget);
        
        _statusPage = new QWidget();
        _statusPage->setStyleSheet("background-color: black;");
        
        QVBoxLayout* statusLayout = new QVBoxLayout(_statusPage);
        statusLayout->setAlignment(Qt::AlignCenter);
        
        _statusLabel = new QLabel();
        _statusLabel->setAlignment(Qt::AlignCenter);
        _statusLabel->setStyleSheet("QLabel { color: white; background-color: rgba(0, 0, 0, 180); "
                                    "padding: 15px; border-radius: 5px; font-weight: bold; font-size: 16px; }");
        
        statusLayout->addWidget(_statusLabel);
        _videoStack->addWidget(_statusPage);
        originalLayout->addWidget(_videoStack);
    }
    
    // Replace old play/pause button with QPlayPauseButton
    QPlayPauseButton* playPauseButton = new QPlayPauseButton(this);
    QWidget* oldButton = ui.playPauseButton;
    playPauseButton->setObjectName("playPauseButton");
    playPauseButton->setGeometry(oldButton->geometry());
    playPauseButton->setMinimumSize(oldButton->minimumSize());
    playPauseButton->setMaximumSize(oldButton->maximumSize());
    playPauseButton->setToolTip(oldButton->toolTip());
    playPauseButton->setEnabled(true);

    QHBoxLayout* topLayout = ui.topLayout;
    topLayout->replaceWidget(oldButton, playPauseButton);
    delete oldButton;
    connect(playPauseButton, &QPlayPauseButton::playClicked, this, [this]() { 
        startStream(ui.rtspUrlInput->text()); 
    });
    connect(playPauseButton, &QPlayPauseButton::pauseClicked, this, &RtspPlayerWidget::stopStream);
    _playPauseButton = playPauseButton;
    
    setupEnhancedControls();
    
    _toggleViewButton = new QPushButton("Show Logs", _videoWidget);
    _toggleViewButton->setMaximumWidth(100);
    QHBoxLayout* btnLayout = new QHBoxLayout();
    btnLayout->addStretch();
    btnLayout->addWidget(_toggleViewButton);
    
    _toggleControlsButton = new QPushButton(this);
    _toggleControlsButton->setIcon(style()->standardIcon(QStyle::SP_ArrowUp));
    _toggleControlsButton->setToolTip("Hide Controls");
    _toggleControlsButton->setMaximumWidth(25);
    _toggleControlsButton->setMaximumHeight(25);
    _toggleControlsButton->setFlat(true);
    connect(_toggleControlsButton, &QPushButton::clicked, this, &RtspPlayerWidget::onToggleControls);
    
    btnLayout->addWidget(_toggleControlsButton);
    static_cast<QVBoxLayout*>(_videoWidget->layout())->addLayout(btnLayout);
    
    _controlsContainer = new QWidget(_videoWidget);
    _controlsContainer->setObjectName("controlsContainer");
    QVBoxLayout* videoLayout = static_cast<QVBoxLayout*>(_videoWidget->layout());
    videoLayout->removeItem(topLayout);
    QVBoxLayout* containerLayout = new QVBoxLayout(_controlsContainer);
    containerLayout->setContentsMargins(0, 0, 0, 0);
    containerLayout->addLayout(topLayout);
    videoLayout->insertWidget(0, _controlsContainer);
    
    _logWidget = new QWidget();
    _logLayout = new QVBoxLayout(_logWidget);
    
    _logControlLayout = new QHBoxLayout();
    QPushButton* backToVideoBtn = new QPushButton("Back to Video", _logWidget);
    _debugCheckbox = new QCheckBox("Debug", _logWidget);
    _infoCheckbox = new QCheckBox("Info", _logWidget);
    _warningCheckbox = new QCheckBox("Warning", _logWidget);
    _errorCheckbox = new QCheckBox("Error", _logWidget);
    _clearButton = new QPushButton("Clear", _logWidget);
    
    _debugCheckbox->setChecked(true);
    _infoCheckbox->setChecked(true);
    _warningCheckbox->setChecked(true);
    _errorCheckbox->setChecked(true);
    
    _logControlLayout->addWidget(backToVideoBtn);
    _logControlLayout->addStretch();
    _logControlLayout->addWidget(_debugCheckbox);
    _logControlLayout->addWidget(_infoCheckbox);
    _logControlLayout->addWidget(_warningCheckbox);
    _logControlLayout->addWidget(_errorCheckbox);
    _logControlLayout->addWidget(_clearButton);
    
    _logDisplay = new QTextEdit(_logWidget);
    _logDisplay->setReadOnly(true);
    _logDisplay->setLineWrapMode(QTextEdit::NoWrap);
    _logDisplay->setStyleSheet("background-color: black; color: white; font-family: monospace;");
    
    _logLayout->addLayout(_logControlLayout);
    _logLayout->addWidget(_logDisplay);
    
    _stackedWidget->addWidget(_videoWidget);
    _stackedWidget->addWidget(_logWidget);
    mainLayout->addWidget(_stackedWidget);
    
    connect(_toggleViewButton, &QPushButton::clicked, this, &RtspPlayerWidget::onToggleView);
    connect(backToVideoBtn, &QPushButton::clicked, this, &RtspPlayerWidget::onToggleView);
    
    connect(_debugCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleDebug);
    connect(_infoCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleInfo);
    connect(_warningCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleWarning);
    connect(_errorCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleError);
    connect(_clearButton, &QPushButton::clicked, this, &RtspPlayerWidget::onClearLogs);
    
    connect(&QLogManager::getInstance(), &QLogManager::newLogMessage, 
            this, &RtspPlayerWidget::onNewLogMessage);
    
    _logDisplay->append("Log initialized for RTSP player " + _widgetId);
}

void RtspPlayerWidget::onArucoButtonClicked()
{
    if (!_arucoDetectionClient) {
        LOG_WARNING_TARGET("RtspPlayer", "Cannot activate Aruco detection - no detection manager set", _widgetId.toUtf8().constData());
        QMessageBox::warning(this, "Aruco Detection", "Aruco detection manager not set.");
        _arucoButton->setChecked(false);
        return;
    }
    
    if (!_playerWorkerThread) {
        _playerWorkerThread = std::make_shared<QPlayerWorker>(true);
        connect(_playerWorkerThread.get(), &QPlayerWorker::detectionHandledSuccessfully,
                this, &RtspPlayerWidget::onDetectionHandledSuccessfully);
        connect(_playerWorkerThread.get(), &QPlayerWorker::arucoServerInfoFailed,
                this, &RtspPlayerWidget::onArucoServerInfoFailed);
        LOG_INFO_TARGET("RtspPlayer", "Created player worker thread", _widgetId.toUtf8().constData());
    }

    if (_arucoButton->isChecked()) {
        startArucoDetection();
    } else {
        stopArucoDetection();
    }
}

void RtspPlayerWidget::startArucoDetection()
{
    if (!_playerWorkerThread || !_arucoDetectionClient) {
        LOG_WARNING_TARGET("RtspPlayer", "Cannot start Aruco detection - dependencies not set", _widgetId.toUtf8().constData());
        return;
    }
    
    std::string url = ui.rtspUrlInput->text().toStdString();
    LOG_INFO_TARGET("RtspPlayer", QString("Starting Aruco detection on %1").arg(ui.rtspUrlInput->text()), _widgetId.toUtf8().constData());
    
    _playerWorkerThread->manageDetection(_arucoDetectionClient, url, _tag, true);
    
    if (!_arucoButton->isChecked()) {
        _arucoButton->setChecked(true);
    }
    _arucoButton->setProperty("class", "success");
    _arucoButton->setStyleSheet("background-color: #5cb85c; color: white;");
    _arucoButton->style()->unpolish(_arucoButton);
    _arucoButton->style()->polish(_arucoButton);
}

void RtspPlayerWidget::stopArucoDetection()
{
    if (!_playerWorkerThread || !_arucoDetectionClient) {
        LOG_WARNING_TARGET("RtspPlayer", "Cannot stop Aruco detection - dependencies not set", _widgetId.toUtf8().constData());
        return;
    }
    
    std::string url = ui.rtspUrlInput->text().toStdString();
    LOG_INFO_TARGET("RtspPlayer", QString("Stopping Aruco detection on %1").arg(ui.rtspUrlInput->text()), _widgetId.toUtf8().constData());
    
    _playerWorkerThread->manageDetection(_arucoDetectionClient, url, _tag, false);
    
    if (_arucoButton->isChecked()) {
        _arucoButton->setChecked(false);
    }
    _arucoButton->setProperty("class", "normal");
    _arucoButton->setStyleSheet("");
    _arucoButton->style()->unpolish(_arucoButton);
    _arucoButton->style()->polish(_arucoButton);
}

void RtspPlayerWidget::displayDetectedArucos(const std::vector<uint16_t>& ids)
{
    std::vector<uint16_t> idsToShow = ids;
    if (idsToShow.size() > NBR_IDS_TO_DISPLAY) {
        idsToShow.resize(NBR_IDS_TO_DISPLAY);
    }
    _arucoIdsTextBox->setText("Ids: ");
    for (const auto& id : idsToShow) {
        _arucoIdsTextBox->setText(_arucoIdsTextBox->text() + "  " + QString::number(id));
    }
}

void RtspPlayerWidget::setArucoDetectionManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client)
{
    if (client != nullptr) {
        _arucoDetectionClient = client;
        if (!_playerWorkerThread) {
            _playerWorkerThread = std::make_shared<QPlayerWorker>(true);
            connect(_playerWorkerThread.get(), &QPlayerWorker::detectionHandledSuccessfully,
                    this, &RtspPlayerWidget::onDetectionHandledSuccessfully);
            connect(_playerWorkerThread.get(), &QPlayerWorker::arucoServerInfoFailed,
                    this, &RtspPlayerWidget::onArucoServerInfoFailed);
            LOG_INFO_TARGET("RtspPlayer", "Created player worker thread", _widgetId.toUtf8().constData());
        }
        LOG_INFO_TARGET("RtspPlayer", "Aruco detection manager set", _widgetId.toUtf8().constData());
    } else {
        LOG_WARNING_TARGET("RtspPlayer", "Error, couldn't access aruco detection manager client", _widgetId.toUtf8().constData());
        _arucoButton->setProperty("class", "error");
        _arucoButton->setStyleSheet("background-color: #d9534f; color: white;");
        _arucoButton->style()->unpolish(_arucoButton);
        _arucoButton->style()->polish(_arucoButton);
    }
}

void RtspPlayerWidget::arucoStillAliveUpdate(bool urlFound)
{
    if (!urlFound && _arucoButton->isChecked()) {
        LOG_WARNING_TARGET("RtspPlayer", QString("Error, aruco detection on %1 was not found").arg(ui.rtspUrlInput->text()), _widgetId.toUtf8().constData());
        _arucoButton->setProperty("class", "normal");
        _arucoButton->setStyleSheet("");
        _arucoButton->style()->unpolish(_arucoButton);
        _arucoButton->style()->polish(_arucoButton);
    }
}

void RtspPlayerWidget::onDetectionHandledSuccessfully(bool success, uint16_t tag)
{
    if (!success && _tag == tag) {
        LOG_WARNING_TARGET("RtspPlayer", QString("Error, request made on %1 regarding aruco detection failed").arg(ui.rtspUrlInput->text()), _widgetId.toUtf8().constData());
        _arucoButton->setProperty("class", "error");
        _arucoButton->setStyleSheet("background-color: #d9534f; color: white;");
        _arucoButton->style()->unpolish(_arucoButton);
        _arucoButton->style()->polish(_arucoButton);
    }
}

void RtspPlayerWidget::onArucoServerInfoFailed(bool success)
{
    if (!success) {
        LOG_WARNING_TARGET("RtspPlayer", "Error, info request to aruco detection manager client failed", _widgetId.toUtf8().constData());
        _arucoButton->setEnabled(false);
    } else {
        if (_arucoButton->property("class") != "success" && _arucoButton->property("class") != "error") {
            _arucoButton->setProperty("class", "normal");
            _arucoButton->setEnabled(true);
            _arucoButton->setStyleSheet("");
            _arucoButton->style()->unpolish(_arucoButton);
            _arucoButton->style()->polish(_arucoButton);
        }
    }
}

void RtspPlayerWidget::onArucoCameraFailed(bool valid)
{
    if (!valid) {
        if (!_arucoButton->isChecked()) {
            _arucoButton->setChecked(true);
        }
        if (!_arucoButton->isEnabled()) {
            _arucoButton->setEnabled(true);
        }
        LOG_WARNING_TARGET("RtspPlayer", QString("Error, camera at %1 is not accessible").arg(ui.rtspUrlInput->text()), _widgetId.toUtf8().constData());
        _arucoButton->setProperty("class", "error");
        _arucoButton->setStyleSheet("background-color: #d9534f; color: white;");
        _arucoButton->style()->unpolish(_arucoButton);
        _arucoButton->style()->polish(_arucoButton);
    }
    if (valid) {
        if (!_arucoButton->isChecked()) {
            _arucoButton->setChecked(true);
        }
        if (!_arucoButton->isEnabled()) {
            _arucoButton->setEnabled(true);
        }
        _arucoButton->setProperty("class", "success");
        _arucoButton->setStyleSheet("background-color: #5cb85c; color: white;");
        _arucoButton->style()->unpolish(_arucoButton);
        _arucoButton->style()->polish(_arucoButton);
    }
}

void RtspPlayerWidget::startStream(const QString& rtspUrl)
{
    if (rtspUrl.isEmpty())
    {
        LOG_WARNING_TARGET("RtspPlayer", "Empty RTSP URL provided", _widgetId.toUtf8().constData());
        return;
    }

    if (!validateRtspUrl(rtspUrl)) {
        QMessageBox::warning(this, "Invalid RTSP URL",
                           "The URL format is invalid. Please enter a valid RTSP URL.\n\n"
                           "Format: rtsp://[username:password@]host[:port]/path");
        return;
    }
    
    QDateTime currentTime = QDateTime::currentDateTime();
    if (rtspUrl == _lastStreamUrl && _lastStreamTime.isValid() && 
        _lastStreamTime.msecsTo(currentTime) < 500) {
        return;
    }
    
    _lastStreamUrl = rtspUrl;
    _lastStreamTime = currentTime;

    if (!inReconnectionMode)
    {
        LOG_INFO_TARGET("RtspPlayer", QString("Starting stream: %1").arg(rtspUrl), _widgetId.toUtf8().constData());
        reconnectAttempts = 0;
        connectionFailed = false;
    }

    receivingFrames = false;
    updateStatusText("Connecting...");
    _playPauseButton->setPlaying(true);
    connectionTimeoutTimer->start(8000);
    emit requestStartStream(rtspUrl);
}

void RtspPlayerWidget::stopStream()
{
    connectionTimeoutTimer->stop();
    
    if (!pipeline && !receivingFrames && !inReconnectionMode)
    {
        return;
    }

    LOG_INFO_TARGET("RtspPlayer", "Stopping stream", _widgetId.toUtf8().constData());
    emit requestStopStream();
    receivingFrames = false;
    inReconnectionMode = false;

    frameTimeoutTimer->stop();
    reconnectTimer->stop();
    
    _arucoButton->setEnabled(false);
    _screenshotButton->setEnabled(false);
    _recordButton->setEnabled(false);
    
    if (!connectionFailed) {
        if (wasEverConnected) {
            updateStatusText("Paused");
        } else {
            updateStatusText("Not Connected");
        }
    }
    
    _playPauseButton->setPlaying(false);
    emitStateChanged();
}

void RtspPlayerWidget::onPipelineStarted(GstElement* receivedPipeline)
{
    if (!receivedPipeline)
    {
        LOG_ERROR_TARGET("RtspPlayer", "Pipeline creation failed", _widgetId.toUtf8().constData());
        updateStatusText("Connection Error");
        _playPauseButton->setPlaying(false);
        connectionTimeoutTimer->stop();
        return;
    }

    pipeline = receivedPipeline;
    GstElement* videoSink = gst_bin_get_by_interface(GST_BIN(pipeline), GST_TYPE_VIDEO_OVERLAY);
    if (!videoSink)
    {
        LOG_ERROR_TARGET("RtspPlayer", "Failed to find VideoOverlay in pipeline", _widgetId.toUtf8().constData());
        updateStatusText("Connection Error");
        _playPauseButton->setPlaying(false);
        connectionTimeoutTimer->stop();
        return;
    }

    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(videoSink), (guintptr)ui.videoWidget->winId());
    gst_object_unref(videoSink);
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    LOG_DEBUG_TARGET("RtspPlayer", "Pipeline state set to PLAYING", _widgetId.toUtf8().constData());
    _playPauseButton->setPlaying(true);
    frameTimeoutTimer->start(2000);
}

void RtspPlayerWidget::onErrorOccurred(const QString& error)
{
    if (!receivingFrames)
    {
        if (inReconnectionMode)
        {
            LOG_DEBUG_TARGET("RtspPlayer", error, _widgetId.toUtf8().constData());
            updateStatusText(QString("Reconnecting... (%1/%2)").arg(reconnectAttempts).arg(maxReconnectAttempts));
            if (!reconnectTimer->isActive()) {
                reconnectTimer->start(3000);
            }
        }
        else
        {
            LOG_ERROR_TARGET("RtspPlayer", error, _widgetId.toUtf8().constData());
            updateStatusText("Connection Error");
            inReconnectionMode = true;
            reconnectAttempts = 0;
            LOG_INFO_TARGET("RtspPlayer", "Attempting reconnection...", _widgetId.toUtf8().constData());
            reconnectTimer->start(3000);
        }
    }
}

void RtspPlayerWidget::updateStatusText(const QString& text)
{
    if (!_statusLabel || !_videoStack)
        return;
    _statusLabel->setText(text);
    _videoStack->setCurrentIndex(text.isEmpty() ? 0 : 1);
}

void RtspPlayerWidget::onNewLogMessage(const QString& message, const QString& target)
{
    if (target == _widgetId)
    {
        _logDisplay->append(message);
        QScrollBar* scrollBar = _logDisplay->verticalScrollBar();
        scrollBar->setValue(scrollBar->maximum());
    }
}

void RtspPlayerWidget::onToggleDebug(bool checked)
{
    QLogManager::getInstance().setShowDebug(checked, _widgetId);
}

void RtspPlayerWidget::onToggleInfo(bool checked)
{
    QLogManager::getInstance().setShowInfo(checked, _widgetId);
}

void RtspPlayerWidget::onToggleWarning(bool checked)
{
    QLogManager::getInstance().setShowWarning(checked, _widgetId);
}

void RtspPlayerWidget::onToggleError(bool checked)
{
    QLogManager::getInstance().setShowError(checked, _widgetId);
}

void RtspPlayerWidget::onClearLogs()
{
    _logDisplay->clear();
    _logDisplay->append("Logs cleared for RTSP player " + _widgetId);
}

void RtspPlayerWidget::onToggleView()
{
    _stackedWidget->setCurrentWidget(_stackedWidget->currentWidget() == _videoWidget ? _logWidget : _videoWidget);
}

void RtspPlayerWidget::onToggleControls()
{
    setControlsVisible(!_controlsVisible);
}

void RtspPlayerWidget::setControlsVisible(bool visible)
{
    _controlsVisible = visible;
    if (_controlsContainer) {
        _controlsContainer->setVisible(visible);
    }
    if (_toggleControlsButton) {
        if (visible) {
            _toggleControlsButton->setIcon(style()->standardIcon(QStyle::SP_ArrowUp));
            _toggleControlsButton->setToolTip("Hide Controls");
        } else {
            _toggleControlsButton->setIcon(style()->standardIcon(QStyle::SP_ArrowDown));
            _toggleControlsButton->setToolTip("Show Controls");
        }
    }
    emit controlsVisibilityChanged(visible);
}

void RtspPlayerWidget::emitStateChanged()
{
    emit streamStateChanged(receivingFrames, _streamIndex);
}

bool RtspPlayerWidget::validateRtspUrl(const QString& url)
{
    static QRegularExpression rtspRegex(
        "^rtsp://(?:([^:@]+)(?::([^@]+))?@)?([^:/]+)(?::(\\d+))?(/.*)?$",
        QRegularExpression::CaseInsensitiveOption
    );
    
    QRegularExpressionMatch match = rtspRegex.match(url);
    if (!match.hasMatch()) {
        LOG_WARNING_TARGET("RtspPlayer", "Invalid RTSP URL format: " + url, _widgetId.toUtf8().constData());
        return false;
    }
    LOG_DEBUG_TARGET("RtspPlayer", "Valid RTSP URL: " + url, _widgetId.toUtf8().constData());
    return true;
}

void RtspPlayerWidget::updateUrlValidationUI(bool isValid)
{
    if (isValid) {
        ui.rtspUrlInput->setStyleSheet("QLineEdit { border: 1px solid #5cb85c; }");
        ui.rtspUrlInput->setToolTip("Valid RTSP URL");
    } else {
        ui.rtspUrlInput->setStyleSheet("QLineEdit { border: 1px solid #d9534f; }");
        ui.rtspUrlInput->setToolTip("Invalid RTSP URL format.\nExpected: rtsp://[username:password@]host[:port]/path");
    }
}
