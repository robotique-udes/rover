#include "QRtspPlayer/QRtspPlayerWidgetHeader.hpp"
#include "QLoggingMacros.hpp"
#include <QStyle>
#include <QPainter>
#include <QMessageBox>

void RtspPlayerWidget::setupUI(void)
{
    this->storeUIReferences();
    this->connectUISignals();
    this->initializeUIState();
}

void RtspPlayerWidget::storeUIReferences(void)
{
    _stackedWidget = _ui->mainStackedWidget;
    _videoWidget = _ui->videoWidget;
    _videoStack = _ui->videoStack;
    _statusPage = _ui->statusPage;
    _statusLabel = _ui->statusLabel;
    
    // Control buttons
    _playPauseButton = _ui->playPauseButton;
    _arucoButton = _ui->arucoButton;
    _arucoIdsTextBox = _ui->arucoIdsTextBox;
    _screenshotButton = _ui->screenshotButton;
    _recordButton = _ui->recordButton;
    _toggleControlsButton = _ui->toggleControlsButton;
    _toggleViewButton = _ui->toggleViewButton;
    _controlsContainer = _ui->controlsContainer;
    _streamSelector = _ui->streamSelector;
    _rtspUrlInput = _ui->rtspUrlInput;
    
    // Log view widgets
    _logWidget = _ui->logWidget;
    _logDisplay = _ui->logDisplay;
    _debugCheckbox = _ui->debugCheckbox;
    _infoCheckbox = _ui->infoCheckbox;
    _warningCheckbox = _ui->warningCheckbox;
    _errorCheckbox = _ui->errorCheckbox;
    _clearButton = _ui->clearButton;
}

void RtspPlayerWidget::connectUISignals(void)
{
    if (this->_playPauseButton) {
        connect(this->_playPauseButton, &QPushButton::clicked, this, [this]() {
            if (this->_playPauseButton->isChecked()) {
                if (this->_rtspUrlInput) {
                    this->startStream(this->_rtspUrlInput->text());
                }
                this->_playPauseButton->setIcon(QIcon(":/icons/stop.png"));
                this->_playPauseButton->setToolTip("Stop");
            } else {
                this->stopStream();
                this->_playPauseButton->setIcon(QIcon(":/icons/play.png"));
                this->_playPauseButton->setToolTip("Play");
            }
        });
    }
    
    // Connect the rest of the signals with similar safety checks
    if (this->_arucoButton) {
        connect(this->_arucoButton, &QPushButton::toggled, this, &RtspPlayerWidget::onArucoButtonToggled);
    }
    
    if (this->_screenshotButton) {
        connect(this->_screenshotButton, &QToolButton::clicked, this, &RtspPlayerWidget::onScreenshotButtonClicked);
    }
    
    if (this->_recordButton) {
        connect(this->_recordButton, &QToolButton::toggled, this, &RtspPlayerWidget::onRecordButtonToggled);
    }
    
    if (this->_toggleViewButton) {
        connect(this->_toggleViewButton, &QPushButton::clicked, this, &RtspPlayerWidget::onToggleView);
    }
    
    QPushButton* backToVideoBtn = this->findChild<QPushButton*>("backToVideoBtn");
    if (backToVideoBtn) {
        connect(backToVideoBtn, &QPushButton::clicked, this, &RtspPlayerWidget::onToggleView);
    }
    
    if (this->_toggleControlsButton) {
        connect(this->_toggleControlsButton, &QPushButton::clicked, this, &RtspPlayerWidget::onToggleControls);
    }
    
    if (this->_streamSelector) {
        connect(this->_streamSelector, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &RtspPlayerWidget::onStreamSelected);
    }
    
    if (this->_rtspUrlInput) {
        connect(this->_rtspUrlInput, &QLineEdit::textChanged, this, &RtspPlayerWidget::onUrlTextChanged);
    }
    
    if (this->_debugCheckbox) {
        connect(this->_debugCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleDebug);
    }
    
    if (this->_infoCheckbox) {
        connect(this->_infoCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleInfo);
    }
    
    if (this->_warningCheckbox) {
        connect(this->_warningCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleWarning);
    }
    
    if (this->_errorCheckbox) {
        connect(this->_errorCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleError);
    }
    
    if (this->_clearButton) {
        connect(this->_clearButton, &QPushButton::clicked, this, &RtspPlayerWidget::onClearLogs);
    }
    
    connect(&QLogManager::getInstance(), &QLogManager::newLogMessage, 
            this, &RtspPlayerWidget::onNewLogMessage);
}

void RtspPlayerWidget::initializeUIState(void)
{
    this->updateStatusText("Not Connected");
    
    this->_controlsVisible = true;
    
    // Make sure QLogManager is initialized before using it
    QLogManager& logManager = QLogManager::getInstance();
    logManager.setShowDebug(false, this->_widgetId);
    logManager.setShowInfo(true, this->_widgetId);
    logManager.setShowWarning(true, this->_widgetId);
    logManager.setShowError(true, this->_widgetId);
    
    // Only add the log message if logDisplay exists
    if (this->_logDisplay) {
        this->_logDisplay->append("Log initialized for RTSP player " + this->_widgetId);
    }
    
    // Now log info in a way that won't try to use the log UI widget
    qDebug() << "Log initialized for RTSP player" << this->_widgetId;
    
    // Initialize button states only if they exist
    if (this->_arucoButton) {
        this->_arucoButton->setEnabled(false);
        this->_arucoButton->setCheckable(true);
    }
    
    if (this->_arucoIdsTextBox) {
        this->_arucoIdsTextBox->setText("Ids: ");
    }
    
    if (this->_screenshotButton) {
        this->_screenshotButton->setEnabled(false);
    }
    
    if (this->_recordButton) {
        this->_recordButton->setEnabled(false);
    }
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
    
    if (this->_controlsContainer) {
        this->_controlsContainer->setVisible(visible_);
    }
    
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
    if (index <= 0) {
        this->_rtspUrlInput->setText("");
    } else if (_predefinedStreams.size() >= static_cast<size_t>(index)) {
        int predefinedIndex = index - 1;
        this->_rtspUrlInput->setText(_predefinedStreams[predefinedIndex].url);
    }
}

void RtspPlayerWidget::addPredefinedStream(const QString& name, const QString& url)
{
    PredefinedStream stream;
    stream.name = name;
    stream.url = url;
    _predefinedStreams.push_back(stream);
    
    if (_streamSelector) {
        _streamSelector->addItem(name);
    }
}
