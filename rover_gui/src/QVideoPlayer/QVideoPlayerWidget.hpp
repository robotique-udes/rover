#ifndef QVIDEOPLAYERWIDGER_HPP
#define QVIDEOPLAYERWIDGER_HPP

#include "rclcpp/rclcpp.hpp"
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <QtWidgets/QCheckBox>  
#include <QtWidgets/QTextEdit> 
#include <QThread>  
#include <QTimer>
#include <QDateTime>
#include "UI_VideoPlayer.h"
#include "Worker/QPlayerWorker.hpp"
#include "Worker/QGStreamerWorker.hpp"
#include <gst/gst.h>

class QVideoPlayerWidget : public QWidget
{
    Q_OBJECT

    static constexpr size_t DELAY_OPENING_CAM_RETRY_MS = 5'000UL;
    static constexpr size_t MAX_DELAY_SERVICE_CALL = 2'000UL;
    static constexpr size_t NBR_IDS_TO_DISPLAY = 5U;
    static constexpr int MAX_RECONNECT_ATTEMPTS = 3;
    static int _instanceCounter;

  public:
    enum class PlayerState
    {
        NotConnected,
        Connecting,
        Streaming,
        Reconnecting,
        Paused,
        ConnectionError,
        ConnectionFailed
    };

    QVideoPlayerWidget(std::shared_ptr<rclcpp::Node> guiNode_,
                       std::string url_,
                       uint16_t tag_,
                       std::shared_ptr<QPlayerWorker> worker_);

    ~QVideoPlayerWidget();

    void setArucoClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_);
    void startDetection(void);
    void stopDetection(void);
    void handleArucoDetection(void);
    void arucoStillAliveUpdate(bool urlFound_);
    void displayDetectedArucos(std::vector<uint16_t> ids_);

    void startStream(const QString& rtspUrl_);
    void stopStream(void);
    void setPlayerState(PlayerState state_);
    void tryReconnect(void);
    void updateStatusText(const QString& text_);
    bool validateRtspUrl(const QString& url_);
    void updateUrlValidationUI(bool isValid_);

    void handlePlayPauseButton(void);

    std::string getCamURL(void);
    void setCamURL(std::string newCamUrl_);
    void setURLToDefault(void);
    void updateCamURL(void);

    QString getId(void) const
    {
        return _widgetId;
    }
    bool isStreaming(void) const
    {
        return _state == PlayerState::Streaming;
    }

  signals:
    void arucoCameraFailure(bool valid_);
    void streamStateChanged(bool isRunning_, int streamIndex_);
    void requestStartStream(const QString& rtspUrl);
    void requestStopStream();

  private slots:
    void onDetectionHandledSuccessfully(bool success_, uint16_t tag_);
    void onArucoServerInfoFailed(bool success_);
    void onArucoCameraFailed(bool valid_);

    void onPipelineStarted(GstElement* pipeline_);
    void onErrorOccurred(const QString& error_);
    void onConnectionFailed(void);
    void onFrameReceived(void);
    void onFrameTimeout(void);
    void onReconnectTimer(void);
    void onConnectionTimeout(void);
    void onToggleView(void);
    void onUrlTextChanged(const QString& text_);
    
    // Log window functionality
    void clearLogs();
    void toggleLogView(bool show);

  private:
    void setupUI(void);
    void storeUIReferences(void);
    void connectUISignals(void);
    void initializeUIState(void);
    void emitStateChanged(void);
    void cleanupResources(void);
    void logToWidget(const QString& level, const QString& message);

    std::shared_ptr<rclcpp::Node> _node;
    Ui::VideoPlayer _ui;

    std::string _camURL = "";
    std::string _defaultCamUrl = "";
    uint16_t _tag;
    int _streamIndex;
    QString _widgetId;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> _client_arucoManager;
    std::shared_ptr<QPlayerWorker> _playerWorkerThread;

    PlayerState _state = PlayerState::NotConnected;
    int _reconnectAttempts = 0;
    bool _wasEverConnected = false;
    bool _controlsVisible = true;
    GstElement* _pipeline = nullptr;
    QThread* _gstreamerThread = nullptr;
    GStreamerWorker* _gstreamerWorker = nullptr;
    QDateTime _lastStreamTime;

    QTimer _reconnectTimer;
    QTimer _frameTimeoutTimer;
    QTimer _connectionTimeoutTimer;

    QStackedWidget* _stackedWidget = nullptr;
    QWidget* _videoWidget = nullptr;
    QStackedWidget* _videoStack = nullptr;
    QWidget* _statusPage = nullptr;
    QLabel* _statusLabel = nullptr;
    QPushButton* _playPauseButton = nullptr;
    QPushButton* _toggleViewButton = nullptr;
    QLineEdit* _rtspUrlInput = nullptr;
    QPushButton* _arucoButton = nullptr;
    QLineEdit* _arucoIdsTextBox = nullptr;
    QPushButton* _screenshotButton = nullptr;
    QPushButton* _recordButton = nullptr;
    QTextEdit* _logDisplay = nullptr;
    QPushButton* _clearButton = nullptr;
    QCheckBox* _debugCheckbox = nullptr;
    QCheckBox* _infoCheckbox = nullptr;
    QCheckBox* _warningCheckbox = nullptr;
    QCheckBox* _errorCheckbox = nullptr;
};

#endif  // QVIDEOPLAYERWIDGER_HPP