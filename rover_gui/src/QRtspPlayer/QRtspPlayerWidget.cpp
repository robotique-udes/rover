#include "QRtspPlayerWidget.hpp"
#include "QLoggingMacros.hpp"
#include <QMessageBox>
#include <QScrollBar>
#include <gst/video/videooverlay.h>
#include <QRegularExpression>
#include <QEvent>
#include <QMessageBox>


// Remove URL event filter class - no longer needed

// Initialize static counter
int RtspPlayerWidget::instanceCounter = 0;

RtspPlayerWidget::RtspPlayerWidget(QWidget* parent, const QString& widgetId):
    QWidget(parent),
    workerThread(new QThread(this)),
    gstreamerWorker(new GStreamerWorker()),
    reconnectTimer(new QTimer(this)),
    frameTimeoutTimer(new QTimer(this)),
    pipeline(nullptr),
    receivingFrames(false),
    inReconnectionMode(false)
{
    // Generate a unique ID if not provided
    _widgetId = widgetId.isEmpty() ? QString("rtsp_player_%1").arg(++instanceCounter) : widgetId;
    _streamIndex = instanceCounter - 1; // 0-based index for the stream
    
    // Setup UI elements
    setupUI();

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
        }
    });

    gstreamerWorker->moveToThread(workerThread);
    connect(workerThread, &QThread::finished, gstreamerWorker, &QObject::deleteLater);

    connect(this, &RtspPlayerWidget::requestStartStream, gstreamerWorker, &GStreamerWorker::startPipeline);
    connect(this, &RtspPlayerWidget::requestStopStream, gstreamerWorker, &GStreamerWorker::stopPipeline);

    connect(gstreamerWorker, &GStreamerWorker::pipelineStarted, this, &RtspPlayerWidget::onPipelineStarted);
    connect(gstreamerWorker, &GStreamerWorker::errorOccurred, this, &RtspPlayerWidget::onErrorOccurred);

    connect(gstreamerWorker,
            &GStreamerWorker::frameReceived,
            this,
            [this]()
            {
                if (!receivingFrames)
                {
                    if (inReconnectionMode)
                    {
                        LOG_INFO_TARGET("RtspPlayer", "Reconnection successful, receiving frames...", _widgetId.toUtf8().constData());
                        inReconnectionMode = false;
                    }
                    else
                    {
                        LOG_INFO_TARGET("RtspPlayer", "Receiving frames...", _widgetId.toUtf8().constData());
                    }
                    receivingFrames = true;

                    // Update the play/pause button state
                    _playPauseButton->setPlaying(true);
                    emitStateChanged();
                }
                reconnectTimer->stop();
                frameTimeoutTimer->start(2000);
                updateStatusIndicator("green");
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
                    updateStatusIndicator("red");
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
                    updateStatusIndicator("yellow");
                    inReconnectionMode = true;

                    if (!ui.rtspUrlInput->text().isEmpty())
                    {
                        startStream(ui.rtspUrlInput->text());
                    }
                }
            });

    workerThread->start();

    updateStatusIndicator("yellow");
    
    LOG_INFO_TARGET("RtspPlayer", "RTSP Player Widget initialized", _widgetId.toUtf8().constData());
    emitStateChanged();
}

RtspPlayerWidget::~RtspPlayerWidget()
{
    stopStream();

    workerThread->quit();
    workerThread->wait();
    // No need to delete ui as it's on the stack now
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
    
    // Add a logs toggle button to the video view
    _toggleViewButton = new QPushButton("Show Logs", _videoWidget);
    _toggleViewButton->setMaximumWidth(100);
    QHBoxLayout* btnLayout = new QHBoxLayout();
    btnLayout->addStretch();
    btnLayout->addWidget(_toggleViewButton);
    
    // Add button to layout
    static_cast<QVBoxLayout*>(_videoWidget->layout())->addLayout(btnLayout);
    
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

    if (!inReconnectionMode)
    {
        LOG_INFO_TARGET("RtspPlayer", QString("Starting stream: %1").arg(rtspUrl), _widgetId.toUtf8().constData());
    }

    receivingFrames = false;
    updateStatusIndicator("yellow");
    emit requestStartStream(rtspUrl);
}

void RtspPlayerWidget::stopStream()
{
    if (!pipeline && !receivingFrames)
    {
        return;
    }

    LOG_INFO_TARGET("RtspPlayer", "Stopping stream", _widgetId.toUtf8().constData());
    emit requestStopStream();
    receivingFrames = false;
    inReconnectionMode = false;

    frameTimeoutTimer->stop();
    reconnectTimer->stop();
    updateStatusIndicator("yellow");
    
    // Make sure the button reflects the correct state
    _playPauseButton->setPlaying(false);
    
    emitStateChanged();
}

void RtspPlayerWidget::onPipelineStarted(GstElement* receivedPipeline)
{
    if (!receivedPipeline)
    {
        LOG_ERROR_TARGET("RtspPlayer", "Pipeline creation failed", _widgetId.toUtf8().constData());
        updateStatusIndicator("red");
        return;
    }

    pipeline = receivedPipeline;

    GstElement* videoSink = gst_bin_get_by_interface(GST_BIN(pipeline), GST_TYPE_VIDEO_OVERLAY);
    if (!videoSink)
    {
        LOG_ERROR_TARGET("RtspPlayer", "Failed to find VideoOverlay in pipeline", _widgetId.toUtf8().constData());
        updateStatusIndicator("red");
        return;
    }

    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(videoSink), (guintptr)ui.videoWidget->winId());
    gst_object_unref(videoSink);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    LOG_DEBUG_TARGET("RtspPlayer", "Pipeline state set to PLAYING", _widgetId.toUtf8().constData());

    receivingFrames = false;
    frameTimeoutTimer->start(2000);
}

void RtspPlayerWidget::onErrorOccurred(const QString& error)
{
    if (!receivingFrames)
    {
        if (inReconnectionMode)
        {
            LOG_DEBUG_TARGET("RtspPlayer", error, _widgetId.toUtf8().constData());
        }
        else
        {
            LOG_ERROR_TARGET("RtspPlayer", error, _widgetId.toUtf8().constData());

            inReconnectionMode = true;
            LOG_INFO_TARGET("RtspPlayer", "Attempting reconnection in background...", _widgetId.toUtf8().constData());
        }

        updateStatusIndicator("yellow");
        _playPauseButton->setPlaying(false);
        reconnectTimer->start(5000);
    }
}

void RtspPlayerWidget::updateStatusIndicator(const QString& color)
{
    ui.statusIndicator->setStyleSheet(QString("QFrame { border-radius: 10px; background-color: %1; }").arg(color));
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