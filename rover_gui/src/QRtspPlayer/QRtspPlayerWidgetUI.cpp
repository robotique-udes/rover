#include "QRtspPlayer/QRtspPlayerWidgetHeader.hpp"
#include "QLoggingMacros.hpp"
#include <QStyle>
#include <QToolButton>
#include <QPainter>
#include <QComboBox>

void RtspPlayerWidget::setupUI(void)
{
    // Load the UI directly from the .ui file
    this->_ui.setupUi(this);
    
    // Create missing elements and setup stacked widgets
    this->setupVideoStack();
    this->setupCustomControls();
    this->setupLogView();
}

void RtspPlayerWidget::setupVideoStack(void)
{
    // Create video stacked widget to handle status display
    if (this->_ui.videoWidget) {
        // Get the parent layout of the video widget
        QVBoxLayout* parentLayout = qobject_cast<QVBoxLayout*>(this->layout());
        int videoWidgetIndex = -1;
        
        // Find the index of the video widget in the layout
        for (int i = 0; i < parentLayout->count(); i++) {
            if (parentLayout->itemAt(i)->widget() == this->_ui.videoWidget) {
                videoWidgetIndex = i;
                break;
            }
        }
        
        // Create stacked widget for video display
        if (videoWidgetIndex >= 0) {
            // Get the original video widget
            QWidget* originalVideo = this->_ui.videoWidget;
            parentLayout->removeWidget(originalVideo);
            
            // Create a new stacked widget
            this->_videoStack = new QStackedWidget(this);
            this->_videoStack->addWidget(originalVideo);
            
            // Create status page
            this->_statusPage = new QWidget();
            this->_statusPage->setStyleSheet("background-color: black;");
            
            QVBoxLayout* statusLayout = new QVBoxLayout(this->_statusPage);
            statusLayout->setAlignment(Qt::AlignCenter);
            
            this->_statusLabel = new QLabel("Not Connected");
            this->_statusLabel->setObjectName("statusLabel"); // Use the style from the UI file
            this->_statusLabel->setAlignment(Qt::AlignCenter);
            
            statusLayout->addWidget(this->_statusLabel);
            this->_videoStack->addWidget(this->_statusPage);
            
            // Insert the stacked widget back at the same position
            parentLayout->insertWidget(videoWidgetIndex, this->_videoStack);
        }
    }
    
    // Create main application stacked widget for video/logs
    this->_stackedWidget = new QStackedWidget(this);
    QLayout* mainLayout = this->layout();
    
    // Move all widgets from the main layout to the stacked widget
    QWidget* mainPage = new QWidget();
    QVBoxLayout* mainPageLayout = new QVBoxLayout(mainPage);
    mainPageLayout->setContentsMargins(0, 0, 0, 0);
    mainPageLayout->setSpacing(0);
    
    // Move all items from the main layout to the main page layout
    while (mainLayout->count() > 0) {
        QLayoutItem* item = mainLayout->takeAt(0);
        if (item->widget()) {
            mainPageLayout->addWidget(item->widget());
        } else if (item->layout()) {
            mainPageLayout->addLayout(item->layout());
        }
        // Don't delete the item as we're moving it
    }
    
    // Add the main page to the stacked widget
    this->_stackedWidget->addWidget(mainPage);
    
    // Add the stacked widget to the main layout
    mainLayout->addWidget(this->_stackedWidget);
    
    // Store reference to the video widget
    this->_videoWidget = this->_ui.videoWidget;
}

void RtspPlayerWidget::setupCustomControls(void)
{
    // Store references to UI elements we need to access later
    this->_playPauseButton = this->_ui.playPauseButton;
    this->_arucoButton = this->_ui.arucoButton;
    this->_arucoIdsTextBox = this->_ui.arucoIdsTextBox;
    this->_screenshotButton = this->_ui.screenshotButton;
    this->_recordButton = this->_ui.recordButton;
    this->_toggleControlsButton = this->_ui.toggleControlsButton;
    this->_toggleViewButton = this->_ui.toggleViewButton;
    this->_controlsContainer = this->_ui.controlsContainer;
    this->_streamSelector = this->findChild<QComboBox*>("streamSelector");
    
    // Connect UI signals
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
    
    // Connect other button signals - only if they exist
    if (this->_arucoButton)
        connect(this->_arucoButton, &QPushButton::clicked, this, &RtspPlayerWidget::onArucoButtonClicked);
    
    if (this->_screenshotButton)
        connect(this->_screenshotButton, &QToolButton::clicked, this, &RtspPlayerWidget::onScreenshotButtonClicked);
    
    if (this->_recordButton)
        connect(this->_recordButton, &QToolButton::toggled, this, &RtspPlayerWidget::onRecordButtonToggled);
    
    if (this->_toggleViewButton)
        connect(this->_toggleViewButton, &QPushButton::clicked, this, &RtspPlayerWidget::onToggleView);
    
    if (this->_toggleControlsButton)
        connect(this->_toggleControlsButton, &QPushButton::clicked, this, &RtspPlayerWidget::onToggleControls);
        
    // Connect stream selector
    if (this->_streamSelector)
        connect(this->_streamSelector, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &RtspPlayerWidget::onStreamSelected);
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

void RtspPlayerWidget::setupLogView(void)
{
    // Create log view widget
    this->_logWidget = new QWidget();
    this->_logLayout = new QVBoxLayout(this->_logWidget);
    this->_logLayout->setContentsMargins(3, 3, 3, 3);
    
    // Create log control bar
    this->_logControlLayout = new QHBoxLayout();
    this->_logControlLayout->setContentsMargins(0, 0, 0, 3);
    this->_logControlLayout->setSpacing(6);
    
    QPushButton* backToVideoBtn = new QPushButton("Back to Video", this->_logWidget);
    this->_debugCheckbox = new QCheckBox("Debug", this->_logWidget);
    this->_infoCheckbox = new QCheckBox("Info", this->_logWidget);
    this->_warningCheckbox = new QCheckBox("Warning", this->_logWidget);
    this->_errorCheckbox = new QCheckBox("Error", this->_logWidget);
    this->_clearButton = new QPushButton("Clear", this->_logWidget);
    
    // Set default states
    this->_debugCheckbox->setChecked(false);
    this->_infoCheckbox->setChecked(true);
    this->_warningCheckbox->setChecked(true);
    this->_errorCheckbox->setChecked(true);
    
    // Arrange log controls
    this->_logControlLayout->addWidget(backToVideoBtn);
    this->_logControlLayout->addStretch();
    this->_logControlLayout->addWidget(this->_debugCheckbox);
    this->_logControlLayout->addWidget(this->_infoCheckbox);
    this->_logControlLayout->addWidget(this->_warningCheckbox);
    this->_logControlLayout->addWidget(this->_errorCheckbox);
    this->_logControlLayout->addWidget(this->_clearButton);
    
    // Create log display
    this->_logDisplay = new QTextEdit(this->_logWidget);
    this->_logDisplay->setReadOnly(true);
    this->_logDisplay->setLineWrapMode(QTextEdit::NoWrap);
    this->_logDisplay->setStyleSheet("background-color: black; color: white; font-family: monospace;");
    
    // Add to layout
    this->_logLayout->addLayout(this->_logControlLayout);
    this->_logLayout->addWidget(this->_logDisplay);
    
    // Add to main stacked widget
    this->_stackedWidget->addWidget(this->_logWidget);
    
    // Connect log filter signals
    connect(backToVideoBtn, &QPushButton::clicked, this, &RtspPlayerWidget::onToggleView);
    connect(this->_debugCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleDebug);
    connect(this->_infoCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleInfo);
    connect(this->_warningCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleWarning);
    connect(this->_errorCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleError);
    connect(this->_clearButton, &QPushButton::clicked, this, &RtspPlayerWidget::onClearLogs);
    
    // Connect to log manager
    connect(&QLogManager::getInstance(), &QLogManager::newLogMessage, 
            this, &RtspPlayerWidget::onNewLogMessage);
    
    // Initial log message
    this->_logDisplay->append("Log initialized for RTSP player " + this->_widgetId);
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

void RtspPlayerWidget::onToggleView(void)
{
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

// Button handler implementations
void RtspPlayerWidget::onScreenshotButtonClicked()
{
    if (_state != PlayerState::Streaming)
    {
        return;
    }
    
    LOG_INFO_TARGET("RtspPlayer", "Taking screenshot", this->_widgetId.toUtf8().constData());
    // Implementation for screenshot functionality goes here
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
        _ui.recordButton->setIcon(QIcon(":/icons/record_on.png"));
        // Implementation for record start goes here
    }
    else
    {
        LOG_INFO_TARGET("RtspPlayer", "Stopping recording", this->_widgetId.toUtf8().constData());
        // Set the recording inactive icon
        _ui.recordButton->setIcon(QIcon(":/icons/record_off.png"));
        // Implementation for record stop goes here
    }
}