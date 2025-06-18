#include "QVideoPlayerWidget.hpp"
#include "Global/Helpers/QSessionFolderManager/QSessionFolderManager.hpp"
#include "QLogManager.hpp"

#include <QStyle>
#include <QDateTime>
#include <QMessageBox>
#include <QScrollBar>
#include <QRegularExpression>
#include <optional>
#include <atomic>

namespace
{

    constexpr size_t CONNECTION_TIMEOUT_MS = 5000;
    constexpr size_t DELAY_OPENING_CAM_RETRY_MS = 5000;
    constexpr size_t MAX_DELAY_SERVICE_CALL_MS = 2000;
    constexpr size_t NBR_IDS_TO_DISPLAY = 5;
    constexpr size_t MAX_RECONNECT_ATTEMPTS = 3UL;
    constexpr size_t STYLE_RESET_TIME_MS = 2000;
    constexpr size_t THROTTLE_RATE_ERROR_MS = 2000;

    std::atomic<size_t> g_instanceCounter{0};
}  

size_t QVideoPlayerWidget::getNextInstanceIndex()
{
    return g_instanceCounter.fetch_add(1);
}

QVideoPlayerWidget::QVideoPlayerWidget(std::shared_ptr<rclcpp::Node> guiNode_,
                                       const std::string& url_,
                                       uint16_t playerIndex_,
                                       std::shared_ptr<QPlayerWorker> workerThreadAruco_,
                                       std::shared_ptr<QPlayerWorker> workerThreadRecording_):
    _node(guiNode_),
    _camURL(url_),
    _streamIndex(getNextInstanceIndex()),
    _playerIndex(playerIndex_),
    _playerWorkerThreadAruco(workerThreadAruco_),
    _playerWorkerThreadRecording(workerThreadRecording_),
    _reconnectTimer(),
    _frameTimeoutTimer(),
    _connectionTimeoutTimer()

{
    _defaultCamUrl = _camURL;
    _ui.setupUi(this);

    this->setupUI();

    // Use smart pointer instead of raw pointer to prevent leaks
    _gstreamerWorker = std::make_unique<GStreamerWorker>();
    _gstreamerWorker->setTargetWidget(_ui.logDisplay);
    _gstreamerWorker->setVideoWidget(_ui.videoWidget);
    _gstreamerWorker->moveToThread(&_gstreamerThread);

    connect(&_gstreamerThread, &QThread::finished, _gstreamerWorker.get(), &QObject::deleteLater);
    connect(this, &QVideoPlayerWidget::requestStartStream, _gstreamerWorker.get(), &GStreamerWorker::startPipeline);
    connect(this, &QVideoPlayerWidget::requestPauseStream, _gstreamerWorker.get(), &GStreamerWorker::pausePipeline);
    connect(this, &QVideoPlayerWidget::requestStopStream, _gstreamerWorker.get(), &GStreamerWorker::stopPipeline);
    connect(_gstreamerWorker.get(), &GStreamerWorker::pipelineStarted, this, &QVideoPlayerWidget::onPipelineStarted);
    connect(_gstreamerWorker.get(), &GStreamerWorker::errorOccurred, this, &QVideoPlayerWidget::onErrorOccurred);
    connect(_gstreamerWorker.get(), &GStreamerWorker::connectionFailed, this, &QVideoPlayerWidget::onConnectionFailed);
    connect(_gstreamerWorker.get(), &GStreamerWorker::frameReceived, this, &QVideoPlayerWidget::onFrameReceived);
    this->hideAngleSelecter();

    connect(_ui.arucoPushButton, &QPushButton::clicked, this, &QVideoPlayerWidget::handleArucoDetection);
    connect(_playerWorkerThreadAruco.get(),
            &QPlayerWorker::detectionHandledSuccessfully,
            this,
            &QVideoPlayerWidget::onDetectionHandledSuccessfully);
    connect(_playerWorkerThreadAruco.get(),
            &QPlayerWorker::arucoServerInfoFailed,
            this,
            &QVideoPlayerWidget::onArucoServerInfoFailed);

    connect(_ui.rtspTextBox, &QLineEdit::textChanged, this, &QVideoPlayerWidget::updateCamURL);
    connect(_ui.defaultStreamPushButton, &QPushButton::clicked, this, &QVideoPlayerWidget::setURLToDefault);
    connect(this, &QVideoPlayerWidget::arucoCameraFailure, this, &QVideoPlayerWidget::onArucoCameraFailed);

    connect(&_frameTimeoutTimer, &QTimer::timeout, this, &QVideoPlayerWidget::onFrameTimeout);
    connect(&_reconnectTimer, &QTimer::timeout, this, &QVideoPlayerWidget::onReconnectTimer);
    connect(&_connectionTimeoutTimer, &QTimer::timeout, this, &QVideoPlayerWidget::onConnectionTimeout);

    connect(&QLogManager::getInstance(), &QLogManager::newLogMessage, this, &QVideoPlayerWidget::onNewLogMessage);

    connect(_ui.ScreenshotButton, &QPushButton::clicked, this, &QVideoPlayerWidget::handleScreenshot);
    connect(_playerWorkerThreadRecording.get(),
            &QPlayerWorker::screenshotHandledSuccessfully,
            this,
            &QVideoPlayerWidget::onScreenshotHandledSuccessfully);

    connect(_ui.startRecordingButton, &QPushButton::clicked, this, &QVideoPlayerWidget::handleRecording);
    connect(_ui.cameraAngleSlider, &QSlider::valueChanged, this, &QVideoPlayerWidget::onCameraAngleSliderChanged);
    connect(_ui.cameraAngleBox, &QDoubleSpinBox::valueChanged, this, &QVideoPlayerWidget::onCameraAngleBoxChanged);

    connect(_playerWorkerThreadRecording.get(),
            &QPlayerWorker::startRecordingHandledSuccessfully,
            this,
            &QVideoPlayerWidget::onStartRecordingHandledSuccessfully);

    connect(_playerWorkerThreadRecording.get(),
            &QPlayerWorker::stopRecordingHandledSuccessfully,
            this,
            &QVideoPlayerWidget::onStopRecordingHandledSuccessfully);

    _ui.rtspTextBox->setText(QString::fromStdString(_camURL));
    _ui.rtspTextBox->setAlignment(Qt::AlignCenter);
    _ui.arucoIdsTextBox->setText("Ids: ");

    this->setPlayerState(ePlayerState::NOT_CONNECTED);

    _gstreamerThread.start();
    std::optional<std::string> optionalSessionFolderPath = QSessionFolderManager::getInstance().getSessionFolderPath();
    if (optionalSessionFolderPath.has_value())
    {
        _sessionFolderPath = *optionalSessionFolderPath;
        if (_sessionFolderPath.empty())
        {
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("No session folder found",
                                                                           "SessionFolderManager returned an empty path",
                                                                           QHelper::QToastNotification::eNotifType::ERROR);
        }
    }
    else
    {
        QHelper::QToastNotification::getInstance().notifyFromAnyThread("No session folder found",
                                                                       "SessionFolderManager couldn't return a valid path",
                                                                       QHelper::QToastNotification::eNotifType::ERROR);
    }
}

QVideoPlayerWidget::~QVideoPlayerWidget()
{
    this->cleanupResources();
}

void QVideoPlayerWidget::cleanupResources(void)
{
    this->stopStream();

    if (_gstreamerThread.isRunning())
    {
        _gstreamerThread.quit();
        if (!_gstreamerThread.wait(1000))
        {
            _gstreamerThread.terminate();
            _gstreamerThread.wait();
        }
    }
}

void QVideoPlayerWidget::setupUI(void)
{
    this->connectUISignals();
    this->initializeUIState();

    _ui.videoWidget->setStyleSheet("background-color: black;");
    _ui.videoWidget->setVisible(true);
    _ui.stackedWidget->setCurrentIndex(0);
    _ui.logDisplay->clear();
    _ui.logDisplay->setAcceptRichText(true);
    _ui.logDisplay->document()->setDefaultStyleSheet(
        "span.debug { color: gray; } span.info { color: white; } span.warning { color: orange; } span.error { color: #FF5555; }");
    _ui.logDisplay->setStyleSheet("font-family: monospace; background-color: #222222;");
}

void QVideoPlayerWidget::connectUISignals(void)
{
    connect(_ui.playPauseButton, &QPushButton::clicked, this, &QVideoPlayerWidget::handlePlayPauseButton);
    connect(_ui.toggleViewButton, &QPushButton::clicked, this, &QVideoPlayerWidget::onToggleView);
    connect(_ui.rtspTextBox, &QLineEdit::textChanged, this, &QVideoPlayerWidget::onUrlTextChanged);

    QPushButton* backToVideoBtn = this->findChild<QPushButton*>("backToVideoBtn");
    if (backToVideoBtn)
    {
        connect(backToVideoBtn,
                &QPushButton::clicked,
                this,
                [this]()
                {
                    _ui.stackedWidget->setCurrentIndex(0);
                });
    }

    QPushButton* clearLogsBtn = findChild<QPushButton*>("clearLogsBtn");
    if (clearLogsBtn)
    {
        connect(clearLogsBtn, &QPushButton::clicked, this, &QVideoPlayerWidget::clearLogs);
    }

    QCheckBox* debugCheckbox = findChild<QCheckBox*>("_debugCheckbox");
    if (debugCheckbox)
    {
        connect(debugCheckbox,
                &QCheckBox::toggled,
                this,
                [this](bool checked)
                {
                    QLogManager::getInstance().setShowDebug(checked, _ui.logDisplay);
                });
    }

    QCheckBox* infoCheckbox = findChild<QCheckBox*>("_infoCheckbox");
    if (infoCheckbox)
    {
        connect(infoCheckbox,
                &QCheckBox::toggled,
                this,
                [this](bool checked)
                {
                    QLogManager::getInstance().setShowInfo(checked, _ui.logDisplay);
                });
    }

    QCheckBox* warningCheckbox = findChild<QCheckBox*>("_warningCheckbox");
    if (warningCheckbox)
    {
        connect(warningCheckbox,
                &QCheckBox::toggled,
                this,
                [this](bool checked)
                {
                    QLogManager::getInstance().setShowWarning(checked, _ui.logDisplay);
                });
    }

    QCheckBox* errorCheckbox = findChild<QCheckBox*>("_errorCheckbox");
    if (errorCheckbox)
    {
        connect(errorCheckbox,
                &QCheckBox::toggled,
                this,
                [this](bool checked)
                {
                    QLogManager::getInstance().setShowError(checked, _ui.logDisplay);
                });
    }
}

void QVideoPlayerWidget::initializeUIState(void)
{
    this->updateStatusText("Not Connected");
    this->_controlsVisible = true;

    _ui.playPauseButton->setChecked(false);
    _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
    _ui.arucoPushButton->setEnabled(false);
    _ui.ScreenshotButton->setEnabled(false);
    _ui.startRecordingButton->setEnabled(false);

    QCheckBox* debugCheckbox = findChild<QCheckBox*>("_debugCheckbox");
    if (debugCheckbox)
    {
        debugCheckbox->setChecked(false);
    }

    QCheckBox* infoCheckbox = findChild<QCheckBox*>("_infoCheckbox");
    if (infoCheckbox)
    {
        infoCheckbox->setChecked(true);
    }

    QCheckBox* warningCheckbox = findChild<QCheckBox*>("_warningCheckbox");
    if (warningCheckbox)
    {
        warningCheckbox->setChecked(true);
    }

    QCheckBox* errorCheckbox = findChild<QCheckBox*>("_errorCheckbox");
    if (errorCheckbox)
    {
        errorCheckbox->setChecked(true);
    }
}

void QVideoPlayerWidget::onNewLogMessage(const QString& message_, QWidget* targetWidget_)
{
    if (targetWidget_ == _ui.logDisplay)
    {
        _ui.logDisplay->append(message_);

        QScrollBar* scrollBar = _ui.logDisplay->verticalScrollBar();
        if (scrollBar)
        {
            scrollBar->setValue(scrollBar->maximum());
        }
    }
}

void QVideoPlayerWidget::startStream(const QString& rtspUrl_)
{
    if (rtspUrl_.isEmpty())
    {
        QHelper::QToastNotification::getInstance().notifyFromAnyThread("Invalid URL",
                                                                       "Empty RTSP URL provided. Please enter a valid RTSP URL.",
                                                                       QHelper::QToastNotification::eNotifType::ERROR);

        _ui.playPauseButton->setChecked(false);
        _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
        return;
    }

    if (!this->validateRtspUrl(rtspUrl_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Invalid RTSP URL: %s", rtspUrl_.toStdString().c_str());
        QMessageBox::warning(this,
                             "Invalid RTSP URL",
                             "The URL format is invalid. Please enter a valid RTSP URL.\n\n"
                             "Format: rtsp://[username:password@]host[:port]/path");
        _ui.playPauseButton->setChecked(false);
        _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
        return;
    }

    QDateTime currentTime = QDateTime::currentDateTime();
    if (rtspUrl_ == QString::fromStdString(_camURL) && _lastStreamTime.isValid() && _lastStreamTime.msecsTo(currentTime) < 500)
    {
        return;
    }

    _lastStreamTime = currentTime;
    _camURL = rtspUrl_.toStdString();

    if (_state != ePlayerState::RECONNECTING)
    {
        RCLCPP_INFO(rclcpp::get_logger("GUI"), "Starting stream: %s", rtspUrl_.toStdString().c_str());
        _reconnectAttempts = 0;
    }

    this->setPlayerState(ePlayerState::CONNECTING);

    emit this->requestStartStream(rtspUrl_);
}

void QVideoPlayerWidget::stopStream(void)
{
    _connectionTimeoutTimer.stop();

    if (_state == ePlayerState::NOT_CONNECTED || _state == ePlayerState::PAUSED)
    {
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("GUI"), "Stopping stream: %s", _camURL.c_str());

    _frameTimeoutTimer.stop();
    _reconnectTimer.stop();

    _ui.arucoPushButton->setEnabled(false);
    _ui.ScreenshotButton->setEnabled(false);
    _ui.startRecordingButton->setEnabled(false);

    emit this->requestStopStream();

    if (_wasEverConnected)
    {
        this->setPlayerState(ePlayerState::PAUSED);
    }
    else
    {
        this->setPlayerState(ePlayerState::NOT_CONNECTED);
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
        case ePlayerState::NOT_CONNECTED:
            this->updateStatusText("Not Connected");
            _ui.playPauseButton->setChecked(false);
            _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            break;

        case ePlayerState::CONNECTING:
            this->updateStatusText("Connecting...");
            _ui.playPauseButton->setChecked(true);
            _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-pause"));
            _connectionTimeoutTimer.start(CONNECTION_TIMEOUT_MS);
            break;

        case ePlayerState::STREAMING:
            this->updateStatusText("");
            _ui.playPauseButton->setChecked(true);
            _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-pause"));
            _wasEverConnected = true;
            _ui.arucoPushButton->setEnabled(true);
            _ui.ScreenshotButton->setEnabled(true);
            _ui.startRecordingButton->setEnabled(true);
            _reconnectAttempts = 0;
            RCLCPP_INFO(rclcpp::get_logger("GUI"), "Stream connected successfully");
            break;

        case ePlayerState::RECONNECTING:
            this->updateStatusText(QString("Reconnecting... (%1/%2)").arg(_reconnectAttempts).arg(MAX_RECONNECT_ATTEMPTS));
            _ui.playPauseButton->setChecked(false);
            _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            if (_ui.arucoPushButton->isChecked())
            {
                RCLCPP_INFO(rclcpp::get_logger("GUI"), "Resetting Aruco button due to stream loss");
                _ui.arucoPushButton->setChecked(false);
                _ui.arucoPushButton->setProperty("class", "normal");
                _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
                _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
                _ui.arucoIdsTextBox->setText("Ids: ");
            }
            _ui.arucoPushButton->setEnabled(false);
            _ui.ScreenshotButton->setEnabled(false);
            _ui.startRecordingButton->setEnabled(false);
            break;

        case ePlayerState::PAUSED:
            this->updateStatusText("Paused");
            _ui.playPauseButton->setChecked(false);
            _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            _ui.arucoPushButton->setEnabled(false);
            _ui.ScreenshotButton->setEnabled(false);
            _ui.startRecordingButton->setEnabled(false);
            break;

        case ePlayerState::CONNECTION_ERROR:
            this->updateStatusText("Connection Error");
            _ui.playPauseButton->setChecked(false);
            _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            _frameTimeoutTimer.stop();
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("GUI"), "Connection error occurred");
            this->tryReconnect();
            break;

        case ePlayerState::CONNECTION_FAILED:
            this->updateStatusText("Connection Failed");
            _ui.playPauseButton->setChecked(false);
            _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
            _ui.arucoPushButton->setEnabled(false);
            _ui.ScreenshotButton->setEnabled(false);
            _ui.startRecordingButton->setEnabled(false);
            _frameTimeoutTimer.stop();
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("GUI"), "Connection failed permanently");
            break;
    }

    bool wasStreaming = (oldState == ePlayerState::STREAMING);
    bool isStreaming = (_state == ePlayerState::STREAMING);

    if (wasStreaming != isStreaming)
    {
        this->emitStateChanged();
    }
}

void QVideoPlayerWidget::tryReconnect(void)
{
    if (_state == ePlayerState::CONNECTION_FAILED)
    {
        return;
    }

    _reconnectAttempts++;

    if (_reconnectAttempts <= MAX_RECONNECT_ATTEMPTS)
    {
        this->setPlayerState(ePlayerState::RECONNECTING);
        _reconnectTimer.start(3000);
    }
    else
    {
        this->setPlayerState(ePlayerState::CONNECTION_FAILED);
    }
}

void QVideoPlayerWidget::updateStatusText(const QString& text_)
{
    _ui.statusLabel->setText(text_);
    _ui.videoStack->setCurrentIndex(text_.isEmpty() ? 0 : 1);
}

bool QVideoPlayerWidget::validateRtspUrl(const QString& url_)
{
    return url_.startsWith("rtsp://") || url_.startsWith("rtspt://") || url_.startsWith("rtsps://");
}

void QVideoPlayerWidget::updateUrlValidationUI(bool isValid_)
{
    if (isValid_)
    {
        _ui.rtspTextBox->setStyleSheet("");
        _ui.rtspTextBox->setToolTip("");
    }
    else
    {
        _ui.rtspTextBox->setStyleSheet("border: 1px solid red;");
        _ui.rtspTextBox->setToolTip("Invalid URL format. Expected: rtsp://[username:password@]host[:port]/path");
    }
}

void QVideoPlayerWidget::emitStateChanged(void)
{
    emit this->streamStateChanged(_state == ePlayerState::STREAMING, _streamIndex);
}

void QVideoPlayerWidget::onToggleView(void)
{
    int currentIndex = _ui.stackedWidget->currentIndex();
    int newIndex = (currentIndex == 0) ? 1 : 0;
    _ui.stackedWidget->setCurrentIndex(newIndex);
}

void QVideoPlayerWidget::onUrlTextChanged(const QString& text_)
{
    if (text_.isEmpty())
    {
        _ui.rtspTextBox->setStyleSheet("");
        _ui.rtspTextBox->setToolTip("Enter RTSP URL...");
    }
    else
    {
        bool isValid = this->validateRtspUrl(text_);
        this->updateUrlValidationUI(isValid);

        if (_state == ePlayerState::CONNECTION_FAILED && isValid)
        {
            this->setPlayerState(ePlayerState::NOT_CONNECTED);
        }
    }
}

void QVideoPlayerWidget::clearLogs(void)
{
    _ui.logDisplay->clear();
}

void QVideoPlayerWidget::toggleLogView(bool show_)
{
    _ui.stackedWidget->setCurrentIndex(show_ ? 1 : 0);
}

void QVideoPlayerWidget::onPipelineStarted(GstElement* pipeline_)
{
    if (!pipeline_)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("GUI"), "Pipeline creation failed");
        _connectionTimeoutTimer.stop();
        this->setPlayerState(ePlayerState::CONNECTION_ERROR);
        return;
    }

    _pipeline = pipeline_;
}

void QVideoPlayerWidget::onErrorOccurred(const QString& error_)
{
    if (_state == ePlayerState::RECONNECTING)
    {
        if (!_reconnectTimer.isActive())
        {
            _reconnectTimer.start(3000);
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Stream error: %s", error_.toStdString().c_str());
        this->setPlayerState(ePlayerState::CONNECTION_ERROR);
    }
}

void QVideoPlayerWidget::onConnectionFailed(void)
{
    _reconnectTimer.stop();
    _connectionTimeoutTimer.stop();

    if (_state != ePlayerState::RECONNECTING)
    {
        _reconnectAttempts = 0;
        this->setPlayerState(ePlayerState::CONNECTION_FAILED);
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("GUI"), "Connection failed permanently");
    }
    else
    {
        this->setPlayerState(ePlayerState::CONNECTION_ERROR);
    }
}

void QVideoPlayerWidget::onFrameReceived(void)
{
    _connectionTimeoutTimer.stop();
    _reconnectTimer.stop();

    if (_state != ePlayerState::STREAMING)
    {
        if (_state == ePlayerState::RECONNECTING)
        {
            _reconnectAttempts = 0;
        }
        else
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("GUI"), "Receiving frames...");
        }

        this->setPlayerState(ePlayerState::STREAMING);

        _ui.arucoPushButton->setEnabled(true);
        _ui.ScreenshotButton->setEnabled(true);
        _ui.startRecordingButton->setEnabled(true);
    }
    _frameTimeoutTimer.stop();
    _frameTimeoutTimer.start(CONNECTION_TIMEOUT_MS);
}

void QVideoPlayerWidget::onFrameTimeout(void)
{
    if (_state == ePlayerState::STREAMING)
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"), "Frame timeout - no frames received");

        _ui.arucoPushButton->setEnabled(false);
        _ui.ScreenshotButton->setEnabled(false);
        _ui.startRecordingButton->setEnabled(false);

        if (_ui.arucoPushButton->isChecked())
        {
            _ui.arucoPushButton->setChecked(false);
            _ui.arucoPushButton->setProperty("class", "normal");
            _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
            _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
            _ui.arucoIdsTextBox->setText("Ids: ");
        }

        this->setPlayerState(ePlayerState::CONNECTION_ERROR);
    }
}

void QVideoPlayerWidget::onReconnectTimer(void)
{
    if (_state == ePlayerState::RECONNECTING)
    {
        if (!_ui.rtspTextBox->text().isEmpty())
        {
            this->startStream(_ui.rtspTextBox->text());
        }
    }
}

void QVideoPlayerWidget::onConnectionTimeout(void)
{
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("GUI"), "Connection timeout - no response from server");

    if (_ui.arucoPushButton->isChecked())
    {
        _ui.arucoPushButton->setChecked(false);
        _ui.arucoPushButton->setProperty("class", "normal");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
        _ui.arucoIdsTextBox->setText("Ids: ");
    }

    _reconnectAttempts = 0;
    this->setPlayerState(ePlayerState::CONNECTION_FAILED);
    emit this->requestStopStream();
}

void QVideoPlayerWidget::handlePlayPauseButton(void)
{
    if (_state == ePlayerState::STREAMING)
    {
        _ui.playPauseButton->setChecked(false);
        _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
        emit this->requestPauseStream();

        _frameTimeoutTimer.stop();
        _connectionTimeoutTimer.stop();
        this->setPlayerState(ePlayerState::PAUSED);

        return;
    }

    if (_state == ePlayerState::PAUSED)
    {
        _ui.playPauseButton->setChecked(true);
        _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-pause"));

        emit this->requestStopStream();

        std::string camURL = _camURL;
        QTimer::singleShot(100,
                           this,
                           [this, camURL]()
                           {
                               this->startStream(QString::fromStdString(_camURL));
                           });

        return;
    }

    _ui.playPauseButton->setChecked(true);
    _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-pause"));
    this->startStream(QString::fromStdString(_camURL));
}

void QVideoPlayerWidget::setArucoClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_)
{
    if (client_)
    {
        this->_client_arucoManager = client_;
    }
    else
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("GUI"), "Error, couldn't access aruco detection manager client");
        _ui.arucoPushButton->setProperty("class", "error");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
}

void QVideoPlayerWidget::startDetection(void)
{
    if (_playerWorkerThreadAruco.get() != nullptr)
    {
        _playerWorkerThreadAruco->manageDetection(_client_arucoManager, _camURL, _playerIndex, true);
    }
    else
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("GUI"), "Error, couldn't access Video Player worker");
    }
}

void QVideoPlayerWidget::stopDetection(void)
{
    if (_playerWorkerThreadAruco.get() != nullptr)
    {
        _playerWorkerThreadAruco->manageDetection(_client_arucoManager, _camURL, _playerIndex, false);
    }
    else
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("GUI"), "Error, couldn't access Video Player worker");
    }
}

void QVideoPlayerWidget::handleArucoDetection(void)
{
    if (_ui.arucoPushButton->isChecked())
    {
        RCLCPP_INFO(rclcpp::get_logger("GUI"), "Starting aruco detection on camera %s", _camURL.c_str());
        this->startDetection();

        if (!_ui.arucoPushButton->isChecked())
        {
            _ui.arucoPushButton->setChecked(true);
        }
        _ui.arucoPushButton->setProperty("class", "success");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("GUI"), "Stopping aruco detection on camera %s", _camURL.c_str());
        this->stopDetection();
        if (_ui.arucoPushButton->isChecked())
        {
            _ui.arucoPushButton->setChecked(false);
        }
        _ui.arucoPushButton->setProperty("class", "normal");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
}

void QVideoPlayerWidget::arucoStillAliveUpdate(bool urlFound_)
{
    if (!urlFound_ && _ui.arucoPushButton->isChecked())
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error, aruco detection on %s was not found", _camURL.c_str());
        _ui.arucoPushButton->setProperty("class", "normal");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
}

void QVideoPlayerWidget::displayDetectedArucos(const std::vector<uint16_t>& ids_)
{
    std::vector<uint16_t> displayIds = ids_;
    size_t nbr_ids_detected = displayIds.size();

    if (nbr_ids_detected > NBR_IDS_TO_DISPLAY)
    {
        displayIds.resize(NBR_IDS_TO_DISPLAY);
    }
    _ui.arucoIdsTextBox->setText("Ids: ");

    for (const auto& id : displayIds)
    {
        _ui.arucoIdsTextBox->setText(_ui.arucoIdsTextBox->text() + "  " + QString::number(id));
    }

    if (!displayIds.empty())
    {
        RCLCPP_INFO(rclcpp::get_logger("GUI"),
                    "Detected aruco markers on camera %s: %s",
                    _camURL.c_str(),
                    _ui.arucoIdsTextBox->text().mid(5).toStdString().c_str());
        _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
    }
    else
    {
        _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-pause"));
    }
}

std::string QVideoPlayerWidget::getCamURL(void)
{
    return this->_camURL;
}

float QVideoPlayerWidget::getCameraAngle(void)
{
    return static_cast<float>(_ui.cameraAngleSlider->value());
}

void QVideoPlayerWidget::setCamURL(const std::string& newCamUrl_)
{
    _camURL = newCamUrl_;
    this->hideAngleSelecter();
}

void QVideoPlayerWidget::setURLToDefault(void)
{
    _camURL = this->_defaultCamUrl;
    _ui.rtspTextBox->setText(QString::fromStdString(_camURL));
    this->hideAngleSelecter();
}

void QVideoPlayerWidget::updateCamURL(void)
{
    _camURL = _ui.rtspTextBox->text().toStdString();
    this->hideAngleSelecter();
}

void QVideoPlayerWidget::onDetectionHandledSuccessfully(bool success_, uint16_t playerIndex_)
{
    if (!success_ && _playerIndex == playerIndex_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, request made on %s regarding aruco detection failed", _camURL.c_str());
        _ui.arucoPushButton->setProperty("class", "error");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
}

void QVideoPlayerWidget::onArucoServerInfoFailed(bool success_)
{
    if (!success_)
    {
        _ui.arucoPushButton->setEnabled(false);
    }
    else
    {
        if (_ui.arucoPushButton->property("class") != "success" && _ui.arucoPushButton->property("class") != "error")
        {
            _ui.arucoPushButton->setProperty("class", "normal");
            _ui.arucoPushButton->setEnabled(true);
            _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
            _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
        }
    }
}

void QVideoPlayerWidget::onArucoCameraFailed(bool valid_)
{
    if (!valid_)
    {
        if (!_ui.arucoPushButton->isChecked())
        {
            _ui.arucoPushButton->setChecked(true);
        }
        if (!_ui.arucoPushButton->isEnabled())
        {
            _ui.arucoPushButton->setEnabled(true);
        }

        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, camera at %s is not accessible", _camURL.c_str());
        _ui.arucoPushButton->setProperty("class", "error");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }

    if (valid_)
    {
        if (!_ui.arucoPushButton->isChecked())
        {
            _ui.arucoPushButton->setChecked(true);
        }
        if (!_ui.arucoPushButton->isEnabled())
        {
            _ui.arucoPushButton->setEnabled(true);
        }
        _ui.arucoPushButton->setProperty("class", "success");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
}

QString QVideoPlayerWidget::getId(void)
{
    return QString("widget_%1").arg(reinterpret_cast<quintptr>(_ui.logDisplay));
}

bool QVideoPlayerWidget::isStreaming(void)
{
    return _state == ePlayerState::STREAMING;
}

void QVideoPlayerWidget::setCameraControlClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_)
{
    if (client_)
    {
        this->_client_cameraControlManager = client_;
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error, couldn't access aruco detection manager client");
        _ui.ScreenshotButton->setProperty("class", "error");
        _ui.ScreenshotButton->style()->unpolish(_ui.ScreenshotButton);
        _ui.ScreenshotButton->style()->polish(_ui.ScreenshotButton);
    }
}

void QVideoPlayerWidget::handleScreenshot(void)
{
    if (_playerWorkerThreadRecording.get() != nullptr)
    {
        _playerWorkerThreadRecording->takeScreenshotManager(_client_cameraControlManager,
                                                            _camURL,
                                                            _playerIndex,
                                                            _sessionFolderPath);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, couldn't access Video Player worker");
    }
    return;
}

void QVideoPlayerWidget::handleRecording(void)
{
    if (_playerWorkerThreadRecording.get() != nullptr)
    {
        if (_ui.startRecordingButton->isChecked())
        {
            _playerWorkerThreadRecording->startRecordingManager(_client_cameraControlManager,
                                                                _camURL,
                                                                _playerIndex,
                                                                _sessionFolderPath);
        }
        else
        {
            _playerWorkerThreadRecording->stopRecordingManager(_client_cameraControlManager,
                                                               _camURL,
                                                               _playerIndex,
                                                               _sessionFolderPath);
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, couldn't access Video Player worker");
    }
    return;
}

void QVideoPlayerWidget::onScreenshotHandledSuccessfully(bool success_, const std::string& status_, uint16_t playerIndex_)
{
    if (playerIndex_ == _playerIndex)
    {
        if (!success_)
        {
            _ui.ScreenshotButton->setProperty("class", "error");
            _ui.ScreenshotButton->style()->unpolish(_ui.ScreenshotButton);
            _ui.ScreenshotButton->style()->polish(_ui.ScreenshotButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Couldn't take screenshot",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::ERROR);
        }
        else
        {
            _ui.ScreenshotButton->setProperty("class", "success");
            _ui.ScreenshotButton->style()->unpolish(_ui.ScreenshotButton);
            _ui.ScreenshotButton->style()->polish(_ui.ScreenshotButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Screenshot taken",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::SUCCESS);
        }

        QTimer::singleShot(STYLE_RESET_TIME_MS,
                           this,
                           [this]()
                           {
                               _ui.ScreenshotButton->setProperty("class", "normal");
                               _ui.ScreenshotButton->style()->unpolish(_ui.ScreenshotButton);
                               _ui.ScreenshotButton->style()->polish(_ui.ScreenshotButton);
                           });
    }
    return;
}

void QVideoPlayerWidget::onStartRecordingHandledSuccessfully(bool success_, const std::string& status_, uint16_t playerIndex_)
{
    if (playerIndex_ == _playerIndex)
    {
        if (!success_)
        {
            _ui.startRecordingButton->setProperty("class", "error");
            _ui.startRecordingButton->style()->unpolish(_ui.startRecordingButton);
            _ui.startRecordingButton->style()->polish(_ui.startRecordingButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Couldn't start video",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::ERROR);

            // reset after timer
            QTimer::singleShot(STYLE_RESET_TIME_MS,
                               this,
                               [this]()
                               {
                                   _ui.startRecordingButton->setProperty("class", "");
                                   _ui.startRecordingButton->style()->unpolish(_ui.startRecordingButton);
                                   _ui.startRecordingButton->style()->polish(_ui.startRecordingButton);
                               });
        }
        else
        {
            _ui.startRecordingButton->setProperty("class", "success");
            _ui.startRecordingButton->setIcon(QIcon::fromTheme("media-playback-stop"));
            _ui.startRecordingButton->style()->unpolish(_ui.startRecordingButton);
            _ui.startRecordingButton->style()->polish(_ui.startRecordingButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Video started",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::SUCCESS);
        }
    }
    return;
}

void QVideoPlayerWidget::onStopRecordingHandledSuccessfully(bool success_, const std::string& status_, uint16_t playerIndex_)
{
    if (playerIndex_ == _playerIndex)
    {
        if (!success_)
        {
            _ui.startRecordingButton->setProperty("class", "error");
            _ui.startRecordingButton->style()->unpolish(_ui.ScreenshotButton);
            _ui.startRecordingButton->style()->polish(_ui.ScreenshotButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Couldn't stop recording",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::ERROR);

            // reset after timer
            QTimer::singleShot(STYLE_RESET_TIME_MS,
                               this,
                               [this]()
                               {
                                   _ui.startRecordingButton->setProperty("class", "normal");
                                   _ui.startRecordingButton->style()->unpolish(_ui.startRecordingButton);
                                   _ui.startRecordingButton->style()->polish(_ui.startRecordingButton);
                               });
        }
        else
        {
            _ui.startRecordingButton->setProperty("class", "normal");
            _ui.startRecordingButton->setIcon(QIcon::fromTheme("media-record"));
            _ui.startRecordingButton->style()->unpolish(_ui.startRecordingButton);
            _ui.startRecordingButton->style()->polish(_ui.startRecordingButton);
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Recording stopped",
                                                                           status_,
                                                                           QHelper::QToastNotification::eNotifType::SUCCESS);
        }
    }
    return;
}

void QVideoPlayerWidget::CB_cameraListUpdate(const std::vector<std::string>& urls_)
{
    for (const auto& url : urls_)
    {
        if (url == _camURL)
        {
            if (!_ui.startRecordingButton->isChecked())
            {
                _ui.startRecordingButton->setChecked(true);
                _ui.startRecordingButton->setProperty("class", "success");
                _ui.startRecordingButton->setIcon(QIcon::fromTheme("media-playback-stop"));
                _ui.startRecordingButton->style()->unpolish(_ui.startRecordingButton);
                _ui.startRecordingButton->style()->polish(_ui.startRecordingButton);
            }
            return;
        }
    }

    // if cam_url wasn't found in vector and we're currently recording
    if (_ui.startRecordingButton->isChecked())
    {
        std::string error_message = "Recording on " + _camURL + " was stopped unexpectedly";
        _ui.startRecordingButton->setChecked(false);
        _ui.startRecordingButton->setIcon(QIcon::fromTheme("media-record"));
        _ui.startRecordingButton->style()->unpolish(_ui.startRecordingButton);
        _ui.startRecordingButton->style()->polish(_ui.startRecordingButton);
        QHelper::QToastNotification::getInstance().notifyFromAnyThread("Recording stopped",
                                                                       error_message,
                                                                       QHelper::QToastNotification::eNotifType::WARNING);
    }
}

void QVideoPlayerWidget::CB_serviceCameraControlAvailable(bool available_)
{
    if (!available_)
    {
        _ui.ScreenshotButton->setEnabled(false);
        _ui.startRecordingButton->setEnabled(false);
        if (_playerIndex == 1)
        {
            RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("GUI"),
                                  *_node->get_clock(),
                                  THROTTLE_RATE_ERROR_MS,
                                  "Error, camera control client is unavailable ");
        }
    }
    else if (!_ui.ScreenshotButton->isEnabled() || !_ui.startRecordingButton->isEnabled())
    {
        _ui.ScreenshotButton->setEnabled(true);
        _ui.startRecordingButton->setEnabled(true);
    }
}

void QVideoPlayerWidget::onCameraAngleSliderChanged(void)
{
    _ui.cameraAngleBox->setValue(_ui.cameraAngleSlider->value());
    float angle = static_cast<float>(_ui.cameraAngleSlider->value());
    emit this->notifyCameraAnglePublisher(_camURL, angle);
}

void QVideoPlayerWidget::onCameraAngleBoxChanged(void)
{
    _ui.cameraAngleSlider->setValue(_ui.cameraAngleBox->value());
    float angle = static_cast<float>(_ui.cameraAngleBox->value());
    emit this->notifyCameraAnglePublisher(_camURL, angle);
}

void QVideoPlayerWidget::hideAngleSelecter(void)
{
    if (_camURL == Constants::CameraInfo::CAMERA_URL_MAP.at("Main")
        || _camURL == Constants::CameraInfo::CAMERA_URL_MAP.at("Antenna"))
    {
        _ui.cameraAngleSlider->show();
        _ui.cameraAngleBox->show();
    }
    else
    {
        _ui.cameraAngleSlider->hide();
        _ui.cameraAngleBox->hide();
    }
}