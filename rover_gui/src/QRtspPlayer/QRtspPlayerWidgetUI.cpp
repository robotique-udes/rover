#include "QRtspPlayer/QRtspPlayerWidgetHeader.hpp"
#include "QLoggingMacros.hpp"
#include <QStyle>
#include <QPainter>
#include <QMessageBox>

void RtspPlayerWidget::setupUI(void)
{
    this->_ui.setupUi(this);
    this->storeUIReferences();
    this->connectUISignals();
    this->initializeUIState();
}

void RtspPlayerWidget::storeUIReferences(void)
{
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
    
    connect(this->_playPauseButton, &QPushButton::clicked, this, [this]() {
        if (this->_playPauseButton->isChecked()) {
          
            this->startStream(this->_ui.rtspUrlInput->text());
            this->_playPauseButton->setIcon(QIcon(":/icons/stop.png"));
            this->_playPauseButton->setToolTip("Stop");
        } else {
         
            this->stopStream();
            this->_playPauseButton->setIcon(QIcon(":/icons/play.png"));
            this->_playPauseButton->setToolTip("Play");
        }
    });
    
    connect(this->_arucoButton, &QPushButton::toggled, this, &RtspPlayerWidget::onArucoButtonToggled);
    
    connect(this->_screenshotButton, &QToolButton::clicked, this, &RtspPlayerWidget::onScreenshotButtonClicked);
    connect(this->_recordButton, &QToolButton::toggled, this, &RtspPlayerWidget::onRecordButtonToggled);
    
    connect(this->_toggleViewButton, &QPushButton::clicked, this, &RtspPlayerWidget::onToggleView);
    connect(this->findChild<QPushButton*>("backToVideoBtn"), &QPushButton::clicked, this, &RtspPlayerWidget::onToggleView);
    
    connect(this->_toggleControlsButton, &QPushButton::clicked, this, &RtspPlayerWidget::onToggleControls);
    
    connect(this->_streamSelector, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &RtspPlayerWidget::onStreamSelected);
    
    connect(this->_ui.rtspUrlInput, &QLineEdit::textChanged, this, &RtspPlayerWidget::onUrlTextChanged);
    
    connect(this->_debugCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleDebug);
    connect(this->_infoCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleInfo);
    connect(this->_warningCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleWarning);
    connect(this->_errorCheckbox, &QCheckBox::toggled, this, &RtspPlayerWidget::onToggleError);
    connect(this->_clearButton, &QPushButton::clicked, this, &RtspPlayerWidget::onClearLogs);
    
    connect(&QLogManager::getInstance(), &QLogManager::newLogMessage, 
            this, &RtspPlayerWidget::onNewLogMessage);
}

void RtspPlayerWidget::initializeUIState(void)
{
    this->updateStatusText("Not Connected");
    
    this->_controlsVisible = true;
    
    QLogManager::getInstance().setShowDebug(false, this->_widgetId);
    QLogManager::getInstance().setShowInfo(true, this->_widgetId);
    QLogManager::getInstance().setShowWarning(true, this->_widgetId);
    QLogManager::getInstance().setShowError(true, this->_widgetId);
    
    LOG_INFO_TARGET("RtspPlayer", "Log initialized for RTSP player " + this->_widgetId, this->_widgetId.toUtf8().constData());
    this->_logDisplay->append("Log initialized for RTSP player " + this->_widgetId);
    
    this->_arucoButton->setEnabled(false);
    this->_arucoButton->setCheckable(true);
    this->_arucoIdsTextBox->setText("Ids: ");
    this->_screenshotButton->setEnabled(false);
    this->_recordButton->setEnabled(false);
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

        this->_ui.rtspUrlInput->setText("");
    } else if (_predefinedStreams.size() >= static_cast<size_t>(index)) {
      
        int predefinedIndex = index - 1;
        this->_ui.rtspUrlInput->setText(_predefinedStreams[predefinedIndex].url);
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