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

// Initialize static counter
int RtspPlayerWidget::instanceCounter = 0;

RtspPlayerWidget::RtspPlayerWidget(QWidget* parent, const QString& widgetId):
    QWidget(parent),
    workerThread(new QThread(this)),
    gstreamerWorker(new GStreamerWorker()),
    reconnectTimer(new QTimer(this)),
    frameTimeoutTimer(new QTimer(this)),
    connectionTimeoutTimer(new QTimer(this)), // New timer for initial connection
    pipeline(nullptr),
    receivingFrames(false),
    inReconnectionMode(false),
    reconnectAttempts(0),
    wasEverConnected(false),
    connectionFailed(false),
    _controlsVisible(true),
    _lastStreamUrl(""),
    _lastStreamTime(QDateTime()),
    _arucoDetectionEnabled(false),
    _playerWorkerThread(new QPlayerWorker(true)),
    _tag(instanceCounter)
{
    // Generate a unique ID if not provided
    _widgetId = widgetId.isEmpty() ? QString("rtsp_player_%1").arg(++instanceCounter) : widgetId;
    _streamIndex = instanceCounter - 1; // 0-based index for the stream
    
    // Initialize lastIds
    for(size_t i=0; i<NBR_IDS_TO_DISPLAY; i++) {
        _lastIds[i] = 65535;
    }
    
    // Setup UI elements
    setupUI();

    // Connect worker thread signals
    connect(_playerWorkerThread.get(), &QPlayerWorker::detectionHandledSuccessfully,
            this, &RtspPlayerWidget::onDetectionHandledSuccessfully);
    connect(_playerWorkerThread.get(), &QPlayerWorker::arucoServerInfoFailed,
            this, &RtspPlayerWidget::onArucoServerInfoFailed);

    // Simple text change connections without autocorrection
    connect(ui.rtspUrlInput, &QLineEdit::textChanged, this, [this](const QString& text) {
        if (text.isEmpty()) {
            // Empty input - reset to default style
            ui.rtspUrlInput->setStyleSheet("");
            ui.rtspUrlInput->setToolTip("Enter RTSP URL...");
        } else {
            // Validate without modifying
            bool isValid = validateRtspUrl(text);
            updateUrlValidationUI(isValid);
            
            // Reset connection failed flag when URL changes
            if (connectionFailed && isValid) {
                connectionFailed = false;
            }
        }
    });

    gstreamerWorker->moveToThread(workerThread);
    connect(workerThread, &QThread::finished, gstreamerWorker, &QObject::deleteLater);

    connect(this, &RtspPlayerWidget::requestStartStream, gstreamerWorker, &GStreamerWorker::startPipeline);
    connect(this, &RtspPlayerWidget::requestStopStream, gstreamerWorker, &GStreamerWorker::stopPipeline);

    connect(gstreamerWorker, &GStreamerWorker::pipelineStarted, this, &RtspPlayerWidget::onPipelineStarted);
    connect(gstreamerWorker, &GStreamerWorker::errorOccurred, this, &RtspPlayerWidget::onErrorOccurred);
    
    // Connect to the connectionFailed signal from GStreamerWorker
    connect(gstreamerWorker, &GStreamerWorker::connectionFailed, this, [this]() {
        inReconnectionMode = false;
        reconnectAttempts = 0;
        reconnectTimer->stop();
        connectionTimeoutTimer->stop(); // Stop connection timeout timer
        connectionFailed = true;  // Set the connection failed flag
        updateStatusText("Connection Failed");
        _playPauseButton->setPlaying(false);
        LOG_ERROR_TARGET("RtspPlayer", "Connection failed permanently", _widgetId.toUtf8().constData());
    });

    connect(gstreamerWorker,
            &GStreamerWorker::frameReceived,
            this,
            [this]()
            {
                // Stop the connection timeout timer as we've connected successfully
                connectionTimeoutTimer->stop();
                
                if (!receivingFrames)
                {
                    if (inReconnectionMode)
                    {
                        LOG_INFO_TARGET("RtspPlayer", "Reconnection successful, receiving frames...", _widgetId.toUtf8().constData());
                        inReconnectionMode = false;
                        reconnectAttempts = 0; // Reset reconnection attempts counter on success
                    }
                    else
                    {
                        LOG_INFO_TARGET("RtspPlayer", "Receiving frames...", _widgetId.toUtf8().constData());
                    }
                    receivingFrames = true;
                    wasEverConnected = true;  // Mark that we've connected successfully
                    connectionFailed = false; // Reset connection failed flag when successful

                    // Update the play/pause button state
                    _playPauseButton->setPlaying(true);
                    emitStateChanged();
                    
                    // Show video when receiving frames (switch to video page)
                    updateStatusText("");
                    
                    // Enable enhanced control buttons when stream is active
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

                    // Update the play/pause button state
                    _playPauseButton->setPlaying(false);
                    
                    // Start reconnection process automatically
                    inReconnectionMode = true;
                    updateStatusText("Connection Lost");
                    reconnectTimer->start(3000); // Start reconnection timer with shorter timeout
                    
                    // Disable enhanced control buttons when stream is lost
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
                    // Increment reconnection attempt counter
                    reconnectAttempts++;
                    
                    if (reconnectAttempts <= maxReconnectAttempts)
                    {
                        // Still have attempts left
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
                        // Max attempts reached
                        inReconnectionMode = false;
                        connectionFailed = true;  // Set connection failed flag
                        updateStatusText("Connection Failed");
                        _playPauseButton->setPlaying(false);
                        LOG_ERROR_TARGET("RtspPlayer", "Maximum reconnection attempts reached", _widgetId.toUtf8().constData());
                    }
                }
            });
            
    // Setup connection timeout timer (for initial connection)
    connectionTimeoutTimer->setSingleShot(true);
    connect(connectionTimeoutTimer, &QTimer::timeout, this, [this]() {
        LOG_ERROR_TARGET("RtspPlayer", "Connection timeout - no response from server", _widgetId.toUtf8().constData());
        
        // Similar handling as connection failure
        inReconnectionMode = false;
        reconnectAttempts = 0;
        connectionFailed = true;
        updateStatusText("Connection Timeout");
        _playPauseButton->setPlaying(false);
        
        // Stop the pipeline
        emit requestStopStream();
    });
    
    workerThread->start();
    
    // Set initial status text to "Not Connected"
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

    // Ensure thread properly terminates
    if (workerThread) {
        workerThread->requestInterruption();
        workerThread->quit();
        if (!workerThread->wait(1000)) {
            workerThread->terminate(); // Force termination as last resort
        }
    }
}

void RtspPlayerWidget::setupEnhancedControls()
{
    // Find the appropriate layout to add our buttons
    QHBoxLayout* topLayout = ui.topLayout;
    if (!topLayout) {
        LOG_ERROR_TARGET("RtspPlayer", "Could not find top layout to add enhanced controls", _widgetId.toUtf8().constData());
        return;
    }

    // Create Aruco IDs text box
    _arucoIdsTextBox = new QLineEdit(this);
    _arucoIdsTextBox->setReadOnly(true);
    _arucoIdsTextBox->setText("Ids: ");
    _arucoIdsTextBox->setAlignment(Qt::AlignCenter);
    _arucoIdsTextBox->setMinimumWidth(120);

    // Create Aruco detection button
    _arucoButton = new QPushButton(this);
    _arucoButton->setText("Aruco");
    _arucoButton->setCheckable(true);
    _arucoButton->setEnabled(false); // Initially disabled until stream is active
    _arucoButton->setToolTip("Enable Aruco marker detection");
    _arucoButton->setObjectName("arucoPushButton");
    connect(_arucoButton, &QPushButton::clicked, this, &RtspPlayerWidget::onArucoButtonClicked);
    
    // Handle aruco camera failure
    connect(this, &RtspPlayerWidget::arucoCameraFailure, this, &RtspPlayerWidget::onArucoCameraFailed);

    // Create screenshot button
    _screenshotButton = new QPushButton(this);
    _screenshotButton->setText("Screenshot");
    _screenshotButton->setEnabled(false); // Initially disabled until stream is active
    _screenshotButton->setToolTip("Take a screenshot");

    // Create recording button
    _recordButton = new QPushButton(this);
    _recordButton->setText("Record");
    _recordButton->setCheckable(true);
    _recordButton->setEnabled(false); // Initially disabled until stream is active
    _recordButton->setToolTip("Start/stop recording");

    // Add buttons to layout
    topLayout->insertWidget(0, _arucoButton);
    topLayout->insertWidget(1, _arucoIdsTextBox);
    topLayout->insertWidget(2, _screenshotButton);
    topLayout->insertWidget(3, _recordButton);
    
    // Add a stretch to push the play button to the right
    topLayout->insertStretch(4);
}

void RtspPlayerWidget::setupUI()
{
    // Create main layout
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(0);
    
    // Create stacked widget for video/logs
    _stackedWidget = new QStackedWidget(this);
    
    // --- VIDEO VIEW ---
    
    // Create the video widget and setup the UI in it
    _videoWidget = new QWidget();
    ui.setupUi(_videoWidget);
    
    // Remove the status indicator frame
    if (ui.statusIndicator) {
        ui.statusIndicator->hide();
        ui.statusIndicator->setMaximumSize(0, 0);
    }
    
    // Create a stacked widget container where the video widget was
    QWidget* videoContainer = ui.videoWidget->parentWidget();
    QLayout* originalLayout = nullptr;
    
    if (videoContainer) {
        originalLayout = videoContainer->layout();
        
        // Remove the video widget from its parent
        originalLayout->removeWidget(ui.videoWidget);
        
        // Create video/status stacked widget
        _videoStack = new QStackedWidget(videoContainer);
        
        // Add the video widget to the stacked widget
        _videoStack->addWidget(ui.videoWidget);
        
        // Create status page with centered text
        _statusPage = new QWidget();
        _statusPage->setStyleSheet("background-color: black;");
        
        QVBoxLayout* statusLayout = new QVBoxLayout(_statusPage);
        statusLayout->setAlignment(Qt::AlignCenter);
        
        _statusLabel = new QLabel();
        _statusLabel->setAlignment(Qt::AlignCenter);
        _statusLabel->setStyleSheet("QLabel { color: white; background-color: rgba(0, 0, 0, 180); "
                                 "padding: 15px; border-radius: 5px; font-weight: bold; font-size: 16px; }");
        
        statusLayout->addWidget(_statusLabel);
        
        // Add status page to stacked widget
        _videoStack->addWidget(_statusPage);
        
        // Add the stacked widget to the original layout
        originalLayout->addWidget(_videoStack);
    }
    
    // Promote the playPauseButton to our custom class
    QPlayPauseButton* playPauseButton = new QPlayPauseButton(this);
    QWidget* oldButton = ui.playPauseButton;

    // Copy the geometry and other properties
    playPauseButton->setObjectName("playPauseButton");
    playPauseButton->setGeometry(oldButton->geometry());
    playPauseButton->setMinimumSize(oldButton->minimumSize());
    playPauseButton->setMaximumSize(oldButton->maximumSize());
    playPauseButton->setToolTip(oldButton->toolTip());
    playPauseButton->setEnabled(true);

    // Replace the button in the layout
    QHBoxLayout* topLayout = ui.topLayout;
    topLayout->replaceWidget(oldButton, playPauseButton);

    // Delete the old button
    delete oldButton;

    // Connect play/pause button signals
    connect(playPauseButton, &QPlayPauseButton::playClicked, this, [this]() { 
        startStream(ui.rtspUrlInput->text()); 
    });
    connect(playPauseButton, &QPlayPauseButton::pauseClicked, this, &RtspPlayerWidget::stopStream);

    // Store the button pointer for later use
    _playPauseButton = playPauseButton;
    
    // Add enhanced controls
    setupEnhancedControls();
    
    // Add a logs toggle button to the video view
    _toggleViewButton = new QPushButton("Show Logs", _videoWidget);
    _toggleViewButton->setMaximumWidth(100);
    QHBoxLayout* btnLayout = new QHBoxLayout();
    btnLayout->addStretch();
    btnLayout->addWidget(_toggleViewButton);
    
    // Create toggle controls button
    _toggleControlsButton = new QPushButton(this);
    _toggleControlsButton->setIcon(style()->standardIcon(QStyle::SP_ArrowUp));
    _toggleControlsButton->setToolTip("Hide Controls");
    _toggleControlsButton->setMaximumWidth(25);
    _toggleControlsButton->setMaximumHeight(25);
    _toggleControlsButton->setFlat(true);
    connect(_toggleControlsButton, &QPushButton::clicked, this, &RtspPlayerWidget::onToggleControls);
    
    // Add button to layout
    btnLayout->addWidget(_toggleControlsButton);
    
    // Add button layout to main layout
    static_cast<QVBoxLayout*>(_videoWidget->layout())->addLayout(btnLayout);
    
    // Create a container widget for all controls (top bar)
    _controlsContainer = new QWidget(_videoWidget);
    _controlsContainer->setObjectName("controlsContainer");
    
    // Move top layout to controls container
    QVBoxLayout* videoLayout = static_cast<QVBoxLayout*>(_videoWidget->layout());
    videoLayout->removeItem(topLayout);
    
    QVBoxLayout* containerLayout = new QVBoxLayout(_controlsContainer);
    containerLayout->setContentsMargins(0, 0, 0, 0);
    containerLayout->addLayout(topLayout);
    
    // Insert controls container at the top of the video layout
    videoLayout->insertWidget(0, _controlsContainer);
    
    // --- LOG VIEW ---
    
    // Create log widget 
    _logWidget = new QWidget();
    _logLayout = new QVBoxLayout(_logWidget);
    
    // Control bar for log view
    _logControlLayout = new QHBoxLayout();
    
    // Add a return to video button at top of log view
    QPushButton* backToVideoBtn = new QPushButton("Back to Video", _logWidget);
    
    // Log controls
    _debugCheckbox = new QCheckBox("Debug", _logWidget);
    _infoCheckbox = new QCheckBox("Info", _logWidget);
    _warningCheckbox = new QCheckBox("Warning", _logWidget);
    _errorCheckbox = new QCheckBox("Error", _logWidget);
    _clearButton = new QPushButton("Clear", _logWidget);
    
    // Set initial state
    _debugCheckbox->setChecked(true);
    _infoCheckbox->setChecked(true);
    _warningCheckbox->setChecked(true);
    _errorCheckbox->setChecked(true);
    
    // Add controls to top bar
    _logControlLayout->addWidget(backToVideoBtn);
    _logControlLayout->addStretch();
    _logControlLayout->addWidget(_debugCheckbox);
    _logControlLayout->addWidget(_infoCheckbox);
    _logControlLayout->addWidget(_warningCheckbox);
    _logControlLayout->addWidget(_errorCheckbox);
    _logControlLayout->addWidget(_clearButton);
    
    // Create log display
    _logDisplay = new QTextEdit(_logWidget);
    _logDisplay->setReadOnly(true);
    _logDisplay->setLineWrapMode(QTextEdit::NoWrap);
    _logDisplay->setStyleSheet("background-color: black; color: white; font-family: monospace;");
    
    // Add to log layout
    _logLayout->addLayout(_logControlLayout);
    _logLayout->addWidget(_logDisplay);
    
    // Add both views to stacked widget
    _stackedWidget->addWidget(_videoWidget);
    _stackedWidget->addWidget(_logWidget);
    
    // Set stacked widget as main content
    mainLayout->addWidget(_stackedWidget);
    
    // Connect signals
    connect(_toggleViewButton, &QPushButton::clicked, this, &RtspPlayerWidget::onToggleView);
    connect(backToVideoBtn, &QPushButton::clicked, this, &RtspPlayerWidget::onToggleView);
    
    connect(_debugCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleDebug);
    connect(_infoCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleInfo);
    connect(_warningCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleWarning);
    connect(_errorCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleError);
    connect(_clearButton, &QPushButton::clicked, this, &RtspPlayerWidget::onClearLogs);
    
    // Connect to log manager
    connect(&QLogManager::getInstance(), &QLogManager::newLogMessage, 
            this, &RtspPlayerWidget::onNewLogMessage);
    
    // Initial log message
    _logDisplay->append("Log initialized for RTSP player " + _widgetId);
}

void RtspPlayerWidget::onArucoButtonClicked()
{
    if (_arucoButton->isChecked()) {
        startArucoDetection();
    } else {
        stopArucoDetection();
    }
}

void RtspPlayerWidget::startArucoDetection()
{
    if (_playerWorkerThread.get() != nullptr) {
        std::string url = ui.rtspUrlInput->text().toStdString();
        _playerWorkerThread->manageDetection(_arucoDetectionClient, url, _tag, true);
        
        if (!_arucoButton->isChecked()) {
            _arucoButton->setChecked(true);
        }
        _arucoButton->setProperty("class", "success");
        _arucoButton->setStyleSheet("background-color: #5cb85c; color: white;");
        _arucoButton->style()->unpolish(_arucoButton);
        _arucoButton->style()->polish(_arucoButton);
    } else {
        LOG_WARNING_TARGET("RtspPlayer", "Error, couldn't access Video Player worker", _widgetId.toUtf8().constData());
    }
}

void RtspPlayerWidget::stopArucoDetection()
{
    if (_playerWorkerThread.get() != nullptr) {
        std::string url = ui.rtspUrlInput->text().toStdString();
        _playerWorkerThread->manageDetection(_arucoDetectionClient, url, _tag, false);
        
        if (_arucoButton->isChecked()) {
            _arucoButton->setChecked(false);
        }
        _arucoButton->setProperty("class", "normal");
        _arucoButton->setStyleSheet("");
        _arucoButton->style()->unpolish(_arucoButton);
        _arucoButton->style()->polish(_arucoButton);
    } else {
        LOG_WARNING_TARGET("RtspPlayer", "Error, couldn't access Video Player worker", _widgetId.toUtf8().constData());
    }
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

    // Validate URL without modifying it
    if (!validateRtspUrl(rtspUrl)) {
        // Show a warning to the user
        QMessageBox::warning(this, "Invalid RTSP URL",
                           "The URL format is invalid. Please enter a valid RTSP URL.\n\n"
                           "Format: rtsp://[username:password@]host[:port]/path");
        return;
    }
    
    // Prevent duplicate starts of the same URL in quick succession (500ms)
    QDateTime currentTime = QDateTime::currentDateTime();
    if (rtspUrl == _lastStreamUrl && _lastStreamTime.isValid() && 
        _lastStreamTime.msecsTo(currentTime) < 500) {
        // Skip duplicate start
        return;
    }
    
    // Update last stream URL and time
    _lastStreamUrl = rtspUrl;
    _lastStreamTime = currentTime;

    if (!inReconnectionMode)
    {
        LOG_INFO_TARGET("RtspPlayer", QString("Starting stream: %1").arg(rtspUrl), _widgetId.toUtf8().constData());
        // Reset reconnection attempts when manually starting the stream
        reconnectAttempts = 0;
        connectionFailed = false; // Reset failure flag on manual start
    }

    receivingFrames = false;
    updateStatusText("Connecting...");
    
    // Set button to playing state when starting stream
    _playPauseButton->setPlaying(true);
    
    // Start the connection timeout timer (8 seconds for initial connection)
    connectionTimeoutTimer->start(8000);
    
    emit requestStartStream(rtspUrl);
}

void RtspPlayerWidget::stopStream()
{
    // Stop the connection timeout timer
    connectionTimeoutTimer->stop();
    
    if (!pipeline && !receivingFrames && !inReconnectionMode)
    {
        // If we're not streaming and not in reconnection mode, don't do anything
        return;
    }

    LOG_INFO_TARGET("RtspPlayer", "Stopping stream", _widgetId.toUtf8().constData());
    emit requestStopStream();
    receivingFrames = false;
    inReconnectionMode = false;

    frameTimeoutTimer->stop();
    reconnectTimer->stop();
    
    // Disable enhanced control buttons
    _arucoButton->setEnabled(false);
    _screenshotButton->setEnabled(false);
    _recordButton->setEnabled(false);
    
    // Don't change status message if connection has failed
    if (!connectionFailed) {
        // Use appropriate status message based on connection history
        if (wasEverConnected) {
            updateStatusText("Paused");  // Only show "Paused" if we were connected before
        } else {
            updateStatusText("Not Connected");  // Show "Not Connected" if we never connected
        }
    }
    
    // Make sure the button reflects the correct state
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
        connectionTimeoutTimer->stop(); // Stop the connection timeout timer
        return;
    }

    pipeline = receivedPipeline;

    GstElement* videoSink = gst_bin_get_by_interface(GST_BIN(pipeline), GST_TYPE_VIDEO_OVERLAY);
    if (!videoSink)
    {
        LOG_ERROR_TARGET("RtspPlayer", "Failed to find VideoOverlay in pipeline", _widgetId.toUtf8().constData());
        updateStatusText("Connection Error");
        _playPauseButton->setPlaying(false);
        connectionTimeoutTimer->stop(); // Stop the connection timeout timer
        return;
    }

    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(videoSink), (guintptr)ui.videoWidget->winId());
    gst_object_unref(videoSink);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    LOG_DEBUG_TARGET("RtspPlayer", "Pipeline state set to PLAYING", _widgetId.toUtf8().constData());

    // Keep play button in playing state while connecting
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
            
            // Make sure reconnect timer is running
            if (!reconnectTimer->isActive()) {
                reconnectTimer->start(3000);
            }
        }
        else
        {
            LOG_ERROR_TARGET("RtspPlayer", error, _widgetId.toUtf8().constData());
            updateStatusText("Connection Error");

            inReconnectionMode = true;
            reconnectAttempts = 0; // Reset counter for new connection attempt series
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
    
    // Switch to appropriate stacked widget page
    if (text.isEmpty()) {
        // Empty text means we're showing video
        _videoStack->setCurrentIndex(0); // First page is video
    } else {
        // Non-empty text means showing status
        _videoStack->setCurrentIndex(1); // Second page is status
    }
}

void RtspPlayerWidget::onNewLogMessage(const QString& message, const QString& target)
{
    // Only process messages for this widget's target
    if (target == _widgetId)
    {
        _logDisplay->append(message);
        
        // Auto-scroll to the bottom
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
    // Toggle between video and log views
    if (_stackedWidget->currentWidget() == _videoWidget) {
        _stackedWidget->setCurrentWidget(_logWidget);
    } else {
        _stackedWidget->setCurrentWidget(_videoWidget);
    }
}

void RtspPlayerWidget::onToggleControls()
{
    // Toggle controls visibility
    setControlsVisible(!_controlsVisible);
}

void RtspPlayerWidget::setControlsVisible(bool visible)
{
    _controlsVisible = visible;
    
    // Show/hide controls container
    if (_controlsContainer) {
        _controlsContainer->setVisible(visible);
    }
    
    // Update toggle button icon and tooltip
    if (_toggleControlsButton) {
        if (visible) {
            _toggleControlsButton->setIcon(style()->standardIcon(QStyle::SP_ArrowUp));
            _toggleControlsButton->setToolTip("Hide Controls");
        } else {
            _toggleControlsButton->setIcon(style()->standardIcon(QStyle::SP_ArrowDown));
            _toggleControlsButton->setToolTip("Show Controls");
        }
    }
    
    // Emit signal about visibility change
    emit controlsVisibilityChanged(visible);
}

void RtspPlayerWidget::emitStateChanged()
{
    emit streamStateChanged(receivingFrames, _streamIndex);
}

bool RtspPlayerWidget::validateRtspUrl(const QString& url)
{
    // Basic RTSP URL pattern:
    // rtsp://[username:password@]host[:port]/path
    static QRegularExpression rtspRegex(
        "^rtsp://(?:([^:@]+)(?::([^@]+))?@)?([^:/]+)(?::(\\d+))?(/.*)?$",
        QRegularExpression::CaseInsensitiveOption
    );
    
    QRegularExpressionMatch match = rtspRegex.match(url);
    
    if (!match.hasMatch()) {
        LOG_WARNING_TARGET("RtspPlayer", "Invalid RTSP URL format: " + url, _widgetId.toUtf8().constData());
        return false;
    }
    
    // URL is valid
    LOG_DEBUG_TARGET("RtspPlayer", "Valid RTSP URL: " + url, _widgetId.toUtf8().constData());
    return true;
}

void RtspPlayerWidget::updateUrlValidationUI(bool isValid)
{
    if (isValid) {
        // Valid URL - green border
        ui.rtspUrlInput->setStyleSheet("QLineEdit { border: 1px solid #5cb85c; }");
        ui.rtspUrlInput->setToolTip("Valid RTSP URL");
    } else {
        // Invalid URL - red border
        ui.rtspUrlInput->setStyleSheet("QLineEdit { border: 1px solid #d9534f; }");
        ui.rtspUrlInput->setToolTip("Invalid RTSP URL format.\nExpected: rtsp://[username:password@]host[:port]/path");
    }
}