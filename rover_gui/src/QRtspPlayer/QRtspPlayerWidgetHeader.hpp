#ifndef RTSPPLAYERWIDGET_HPP
#define RTSPPLAYERWIDGET_HPP

#include <QThread>
#include <QTimer>
#include <QWidget>
#include <QTextEdit>
#include <QCheckBox>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QStackedWidget>
#include <QLabel>
#include <QDateTime>
#include <QLineEdit>
#include <QToolButton>
#include <QComboBox>
#include <memory>
#include <vector>
#include <gst/gst.h>

#include "QGStreamerWorker.hpp"
#include "UI_Player.h"
#include "QLogManager.hpp"
#include "QPlayPauseButton.hpp" 
#include "QArrucoWorker/QPlayerWorker.hpp"

enum class PlayerState {
    NotConnected,  
    Connecting,    
    Streaming,     
    Reconnecting,  
    Paused,        
    ConnectionError, 
    ConnectionFailed 
};

class RtspPlayerWidget : public QWidget
{
    Q_OBJECT

public:
    static constexpr uint16_t NBR_IDS_TO_DISPLAY = 5U;
    static constexpr int MAX_RECONNECT_ATTEMPTS = 3;

private:
    static int _instanceCounter;

public:
    explicit RtspPlayerWidget(QWidget* parent_ = nullptr, const QString& widgetId_ = QString());
    ~RtspPlayerWidget();

    void startStream(const QString& rtspUrl_);
    void stopStream(void);
    
    QString getId(void) const { return _widgetId; }
    bool isStreaming(void) const { return _state == PlayerState::Streaming; }
    
    bool validateRtspUrl(const QString& url_);
    void updateUrlValidationUI(bool isValid_);
   
    void setControlsVisible(bool visible_);
    bool areControlsVisible(void) const { return _controlsVisible; }

    // Add a predefined stream to the dropdown
    void addPredefinedStream(const QString& name, const QString& url);

    void startArucoDetection(void);
    void stopArucoDetection(void);
    void displayDetectedArucos(const std::vector<uint16_t>& ids_);
    void setArucoDetectionManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_);
    void arucoStillAliveUpdate(bool urlFound_);

private slots:
   
    void onPipelineStarted(GstElement* pipeline_);
    void onErrorOccurred(const QString& error_);
    void onStreamSelected(int index); // Handler for stream selection
    
    void onConnectionFailed(void);
    void onFrameReceived(void);
    void onFrameTimeout(void);
    void onReconnectTimer(void);
    void onConnectionTimeout(void);
    void onUrlTextChanged(const QString& text_);
 
    void onNewLogMessage(const QString& message_, const QString& target_);
    void onToggleDebug(bool checked_);
    void onToggleInfo(bool checked_);
    void onToggleWarning(bool checked_);
    void onToggleError(bool checked_);
    void onClearLogs(void);
   
    void onToggleView(void);
    void onToggleControls(void);
    
    void onScreenshotButtonClicked();
    void onRecordButtonToggled(bool checked);

    void onArucoButtonClicked(void);
    void onDetectionHandledSuccessfully(bool success_, uint16_t tag_);
    void onArucoServerInfoFailed(bool success_);
    void onArucoCameraFailed(bool valid_);
    
signals:
    void requestStartStream(const QString& rtspUrl_);
    void requestStopStream(void);
    void streamStateChanged(bool isRunning_, int streamIndex_);
    void controlsVisibilityChanged(bool visible_);
    void arucoCameraFailure(bool valid_);

private:
    
    void setPlayerState(PlayerState state_);
    void updateStatusText(const QString& text_);
    void emitStateChanged(void);
    
    void setupUI(void);
    void setupVideoStack(void);
    void setupCustomControls(void);
    void setupLogView(void);
    void connectSignals(void);
    
    void setArucoButtonStyle(const QString& styleClass_, const QString& bgColor_ = "");
    void tryReconnect(void);
    void cleanupResources(void);

    QString _widgetId;
    int _streamIndex;
    Ui::RtspPlayerWidget _ui;

    QThread* _workerThread;
    GStreamerWorker* _gstreamerWorker;

    QTimer* _reconnectTimer;
    QTimer* _frameTimeoutTimer;
    QTimer* _connectionTimeoutTimer;

    QString _lastStreamUrl;
    QDateTime _lastStreamTime;
    GstElement* _pipeline;

    PlayerState _state;
    int _reconnectAttempts;
    bool _wasEverConnected;
    bool _controlsVisible;
    
    // Stream selection
    struct PredefinedStream {
        QString name;
        QString url;
    };
    std::vector<PredefinedStream> _predefinedStreams;
    QComboBox* _streamSelector = nullptr;

    QPushButton* _arucoButton = nullptr;
    QLineEdit* _arucoIdsTextBox = nullptr;
    bool _arucoDetectionEnabled;
    uint16_t _lastIds[NBR_IDS_TO_DISPLAY];
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> _arucoDetectionClient;
    std::shared_ptr<QPlayerWorker> _playerWorkerThread;
    uint16_t _tag;

    QStackedWidget* _stackedWidget = nullptr;
    QWidget* _videoWidget = nullptr;
    QStackedWidget* _videoStack = nullptr;
    QWidget* _statusPage = nullptr;
    QLabel* _statusLabel = nullptr;
    QPlayPauseButton* _playPauseButton = nullptr;
    QPushButton* _toggleControlsButton = nullptr;
    QWidget* _controlsContainer = nullptr;
    QToolButton* _screenshotButton = nullptr;
    QToolButton* _recordButton = nullptr;
    
    QWidget* _logWidget = nullptr;
    QVBoxLayout* _logLayout = nullptr;
    QHBoxLayout* _logControlLayout = nullptr;
    QTextEdit* _logDisplay = nullptr;
    QCheckBox* _debugCheckbox = nullptr;
    QCheckBox* _infoCheckbox = nullptr;
    QCheckBox* _warningCheckbox = nullptr;
    QCheckBox* _errorCheckbox = nullptr;
    QPushButton* _clearButton = nullptr;
    QPushButton* _toggleViewButton = nullptr;
};

#endif