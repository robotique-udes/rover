#include "QRtspPlayer/QRtspPlayerWidgetHeader.hpp"
#include "QLoggingMacros.hpp"
#include <QStyle>
#include <QPainter>
#include <QMessageBox>

void RtspPlayerWidget::setupUI(void)
{
    // Load the UI directly from the .ui file
    this->_ui.setupUi(this);
    
    // Store references to UI elements we'll need to access later
    this->storeUIReferences();
    
    // Connect signals for UI elements
    this->connectUISignals();
    
    // Initialize UI state
    this->initializeUIState();
}

void RtspPlayerWidget::storeUIReferences(void)
{
    // Main stacked widget (for video/logs switch)
    this->_stackedWidget = this->findChild<QStackedWidget*>("mainStackedWidget");
    
    // Video widgets
    this->_videoWidget = this->findChild<QWidget*>("videoWidget");
    this->_videoStack = this->findChild<QStackedWidget*>("videoStack");
    this->_statusPage = this->findChild<QWidget*>("statusPage");
    this->_statusLabel = this->findChild<QLabel*>("statusLabel");
    
    // Control buttons
    this->_playPauseButton = this->findChild<QPushButton*>("playPauseButton");
    this->_arucoButton = this->findChild<QPushButton*>("arucoButton");
    this->_arucoIdsTextBox = this->findChild<QLineEdit*>("arucoIdsTextBox");
    this->_screenshotButton = this->findChild<QToolButton*>("screenshotButton");
    this->_recordButton = this->findChild<QToolButton*>("recordButton");
    this->_toggleControlsButton = this->findChild<QPushButton*>("toggleControlsButton");
    this->_toggleViewButton = this->findChild<QPushButton*>("toggleViewButton");
    this->_controlsContainer = this->findChild<QWidget*>("controlsContainer");
    this->_streamSelector = this->findChild<QComboBox*>("streamSelector");
    
    // Log view widgets
    this->_logWidget = this->findChild<QWidget*>("logWidget");
    this->_logDisplay = this->findChild<QTextEdit*>("logDisplay");
    this->_debugCheckbox = this->findChild<QCheckBox*>("debugCheckbox");
    this->_infoCheckbox = this->findChild<QCheckBox*>("infoCheckbox");
    this->_warningCheckbox = this->findChild<QCheckBox*>("warningCheckbox");
    this->_errorCheckbox = this->findChild<QCheckBox*>("errorCheckbox");
    this->_clearButton = this->findChild<QPushButton*>("clearButton");
}

void RtspPlayerWidget::connectUISignals(void)
{
    // Connect play/pause button signal
    connect(this->_playPauseButton, &QPushButton::clicked, this, [this]() {
        if (this->_playPauseButton->isChecked()) {
            // Button is checked (showing pause icon) - start stream
            this->startStream(this->_ui.rtspUrlInput->text());
            this->_playPauseButton->setIcon(QIcon(":/icons/stop.png"));
            this->_playPauseButton->setToolTip("Stop");
        } else {
            // Button is unchecked (showing play icon) - stop stream
            this->stopStream();
            this->_playPauseButton->setIcon(QIcon(":/icons/play.png"));
            this->_playPauseButton->setToolTip("Play");
        }
    });
    
    // Connect other button signals
    connect(this->_arucoButton, &QPushButton::clicked, this, &RtspPlayerWidget::onArucoButtonClicked);
    connect(this->_screenshotButton, &QToolButton::clicked, this, &RtspPlayerWidget::onScreenshotButtonClicked);
    connect(this->_recordButton, &QToolButton::toggled, this, &RtspPlayerWidget::onRecordButtonToggled);
    
    // View toggling
    connect(this->_toggleViewButton, &QPushButton::clicked, this, &RtspPlayerWidget::onToggleView);
    connect(this->findChild<QPushButton*>("backToVideoBtn"), &QPushButton::clicked, this, &RtspPlayerWidget::onToggleView);
    
    // Controls visibility
    connect(this->_toggleControlsButton, &QPushButton::clicked, this, &RtspPlayerWidget::onToggleControls);
    
    // Stream selector
    connect(this->_streamSelector, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &RtspPlayerWidget::onStreamSelected);
    
    // URL input validation
    connect(this->_ui.rtspUrlInput, &QLineEdit::textChanged, this, &RtspPlayerWidget::onUrlTextChanged);
    
    // Connect log filter signals
    connect(this->_debugCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleDebug);
    connect(this->_infoCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleInfo);
    connect(this->_warningCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleWarning);
    connect(this->_errorCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleError);
    connect(this->_clearButton, &QPushButton::clicked, this, &RtspPlayerWidget::onClearLogs);
    
    // Connect to log manager
    connect(&QLogManager::getInstance(), &QLogManager::newLogMessage, 
            this, &RtspPlayerWidget::onNewLogMessage);
}

void RtspPlayerWidget::initializeUIState(void)
{
    // Set initial state for video stack and status display
    this->updateStatusText("Not Connected");
    
    // Set initial controls visibility
    this->_controlsVisible = true;
    
    // Initialize log filter settings in QLogManager to match checkboxes
    QLogManager::getInstance().setShowDebug(false, this->_widgetId);
    QLogManager::getInstance().setShowInfo(true, this->_widgetId);
    QLogManager::getInstance().setShowWarning(true, this->_widgetId);
    QLogManager::getInstance().setShowError(true, this->_widgetId);
    
    // Initial log message - use INFO level instead of DEBUG
    LOG_INFO_TARGET("RtspPlayer", "Log initialized for RTSP player " + this->_widgetId, this->_widgetId.toUtf8().constData());
    this->_logDisplay->append("Log initialized for RTSP player " + this->_widgetId);
    
    // Initialize active state for stream-dependent controls
    this->_arucoButton->setEnabled(false);
    this->_screenshotButton->setEnabled(false);
    this->_recordButton->setEnabled(false);
}

void RtspPlayerWidget::onToggleView(void)
{
    // Switch between video view (index 0) and log view (index 1)
    int currentIndex = this->_stackedWidget->currentIndex();
    int newIndex = (currentIndex == 0) ? 1 : 0;
    this->_stackedWidget->setCurrentIndex(newIndex);
}

void RtspPlayerWidget::onToggleControls(void)
{
    this->setControlsVisible(!this->_controlsVisible);
}

void RtspPlayerWidget::setControlsVisible(bool visible_)
{
    this->_controlsVisible = visible_;
    
    // Show/hide the controls container
    if (this->_controlsContainer) {
        this->_controlsContainer->setVisible(visible_);
    }
    
    // Update toggle button icon
    if (this->_toggleControlsButton) {
        if (visible_) {
            this->_toggleControlsButton->setIcon(QIcon(":/icons/up_arrow.png"));
            this->_toggleControlsButton->setToolTip("Hide Controls");
        } else {
            this->_toggleControlsButton->setIcon(QIcon(":/icons/down_arrow.png"));
            this->_toggleControlsButton->setToolTip("Show Controls");
        }
    }
    
    emit this->controlsVisibilityChanged(visible_);
}

void RtspPlayerWidget::updateStatusText(const QString& text_)
{
    if (!this->_statusLabel || !this->_videoStack)
    {
        return;
    }
    this->_statusLabel->setText(text_);
    this->_videoStack->setCurrentIndex(text_.isEmpty() ? 0 : 1);
}

void RtspPlayerWidget::onStreamSelected(int index)
{
    // Handle stream selection
    if (index <= 0) {
        // "None" selected - clear URL
        this->_ui.rtspUrlInput->setText("");
    } else if (_predefinedStreams.size() >= static_cast<size_t>(index)) {
        // A predefined stream was selected (index-1 because index 0 is "None")
        int predefinedIndex = index - 1;
        this->_ui.rtspUrlInput->setText(_predefinedStreams[predefinedIndex].url);
    }
}

void RtspPlayerWidget::addPredefinedStream(const QString& name, const QString& url)
{
    // Add to our internal list
    PredefinedStream stream;
    stream.name = name;
    stream.url = url;
    _predefinedStreams.push_back(stream);
    
    // Add to dropdown if it exists
    if (_streamSelector) {
        _streamSelector->addItem(name);
    }
}

// Add implementations for screenshot and record button handlers
void RtspPlayerWidget::onScreenshotButtonClicked()
{
    if (_state != PlayerState::Streaming)
    {
        return;
    }
    
    LOG_INFO_TARGET("RtspPlayer", "Taking screenshot", this->_widgetId.toUtf8().constData());
    
    // Simple placeholder implementation
    QMessageBox::information(this, "Screenshot", 
                           "Screenshot functionality would capture the current frame.\n\n"
                           "This is a placeholder implementation.");
}

void RtspPlayerWidget::onRecordButtonToggled(bool checked)
{
    if (_state != PlayerState::Streaming)
    {
        return;
    }
    
    if (checked)
    {
        LOG_INFO_TARGET("RtspPlayer", "Starting recording", this->_widgetId.toUtf8().constData());
        
        // Set the recording active icon
        _recordButton->setIcon(QIcon(":/icons/record_on.png"));
    }
    else
    {
        LOG_INFO_TARGET("RtspPlayer", "Stopping recording", this->_widgetId.toUtf8().constData());
        
        // Set the recording inactive icon
        _recordButton->setIcon(QIcon(":/icons/record_off.png"));
    }
}