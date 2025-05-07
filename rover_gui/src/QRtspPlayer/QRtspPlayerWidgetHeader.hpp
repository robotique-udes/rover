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
#include "UI_CameraSettings.h"
#include "QLogManager.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/srv/camera_control.hpp"
#include "rover_msgs/srv/aruco_detection.hpp"
#include "rover_msgs/msg/aruco.hpp"

enum class PlayerState {
    NotConnected,  
    Connecting,    
    Streaming,     
    Reconnecting,  
    Paused,        
    ConnectionError, 
    ConnectionFailed 
};

namespace Ui {
    class RtspPlayerWidget;     
    class CameraSettingsWidget; 
}

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

    void addPredefinedStream(const QString& name, const QString& url);
    
    void initializeRosServices(std::shared_ptr<rclcpp::Node> node_);

private slots:
    void onPipelineStarted(GstElement* pipeline_);
    void onErrorOccurred(const QString& error_);
    void onStreamSelected(int index); 
    
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
    
    void onArucoButtonToggled(bool checked);
    void handleScreenshotRequest();
    void handleRecordingRequest(bool checked);
    void onScreenshotButtonClicked() { handleScreenshotRequest(); }
    void onRecordButtonToggled(bool checked) { handleRecordingRequest(checked); }
    
    void checkServiceAvailability();
    
signals:
    void requestStartStream(const QString& rtspUrl_);
    void requestStopStream(void);
    void streamStateChanged(bool isRunning_, int streamIndex_);
    void controlsVisibilityChanged(bool visible_);

private:
    
    void setPlayerState(PlayerState state_);
    void updateStatusText(const QString& text_);
    void emitStateChanged(void);
    
    void setupUI(void);
    void storeUIReferences(void);
    void connectUISignals(void);
    void initializeUIState(void);
    void connectSignals(void);
    
    void tryReconnect(void);
    void cleanupResources(void);
    
    void handleArucoDetection(const std::shared_ptr<rover_msgs::msg::Aruco> msg);
    
    void startServiceAvailabilityPolling();

    QString _widgetId;
    int _streamIndex;
    Ui::RtspPlayerWidget* _ui;
    QWidget* _cameraSettingsWidget;

    // These must remain heap-allocated due to threading
    QThread* _workerThread;                  
    GStreamerWorker* _gstreamerWorker;       

    QTimer _reconnectTimer;
    QTimer _frameTimeoutTimer;
    QTimer _connectionTimeoutTimer;
    
    QTimer _servicePollingTimer;
    bool _servicePollingActive = false;

    QString _lastStreamUrl;
    QDateTime _lastStreamTime;
    GstElement* _pipeline;                   

    PlayerState _state;
    int _reconnectAttempts;
    bool _wasEverConnected;
    bool _controlsVisible;
    bool _arucoDetectionEnabled;
    
    struct PredefinedStream {
        QString name;
        QString url;
    };
    std::vector<PredefinedStream> _predefinedStreams;

    // These are managed by Qt UI framework (from .ui file)
    QStackedWidget* _stackedWidget = nullptr;
    QWidget* _videoWidget = nullptr;
    QStackedWidget* _videoStack = nullptr;
    QWidget* _statusPage = nullptr;
    QLabel* _statusLabel = nullptr;
    QPushButton* _playPauseButton = nullptr;
    QPushButton* _toggleControlsButton = nullptr;
    QPushButton* _toggleViewButton = nullptr;
    QWidget* _controlsContainer = nullptr;
    QToolButton* _screenshotButton = nullptr;
    QToolButton* _recordButton = nullptr;
    QPushButton* _arucoButton = nullptr;
    QLineEdit* _arucoIdsTextBox = nullptr;
    QComboBox* _streamSelector = nullptr;
    QLineEdit* _rtspUrlInput = nullptr;

    QWidget* _logWidget = nullptr;
    QTextEdit* _logDisplay = nullptr;
    QCheckBox* _debugCheckbox = nullptr;
    QCheckBox* _infoCheckbox = nullptr;
    QCheckBox* _warningCheckbox = nullptr;
    QCheckBox* _errorCheckbox = nullptr;
    QPushButton* _clearButton = nullptr;

    std::shared_ptr<rclcpp::Node> _rosNode = nullptr;
    rclcpp::Client<rover_msgs::srv::CameraControl>::SharedPtr _cameraControlClient = nullptr;
    rclcpp::Client<rover_msgs::srv::ArucoDetection>::SharedPtr _arucoDetectionClient = nullptr;
    rclcpp::Subscription<rover_msgs::msg::Aruco>::SharedPtr _arucoSubscription = nullptr;
    bool _isCameraServiceAvailable = false;
    bool _isArucoServiceAvailable = false;
};

#endif  