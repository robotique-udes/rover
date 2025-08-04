#ifndef QVIDEOPLAYERWIDGER_HPP
#define QVIDEOPLAYERWIDGER_HPP

#include "rclcpp/rclcpp.hpp"

#include "UI_VideoPlayer.h"
#include "Worker/QPlayerWorker.hpp"
#include "Worker/QRecordingWorker.hpp"
#include "Worker/QPanoramaWorker.hpp"
#include "Worker/QGStreamerWorker.hpp"
#include "QVideoRecorderWidget.hpp"

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QTextEdit>
#include <QThread>
#include <QTimer>
#include <QDateTime>
#include <gst/gst.h>
#include <Global/Helpers/QToastNotification/QToastNotification.hpp>
#include <rover_lib2/helpers/constants.hpp>

class QVideoPlayerWidget : public QWidget
{
    Q_OBJECT

    static constexpr size_t DELAY_OPENING_CAM_RETRY_MS = 5'000UL;
    static constexpr size_t MAX_DELAY_SERVICE_CALL = 2'000UL;
    static constexpr size_t NBR_IDS_TO_DISPLAY = 5U;

    static int MAX_RECONNECT_ATTEMPTS;
    static int _instanceCounter;

    static constexpr size_t STYLE_RESET_TIME = 2'000UL;

    static constexpr uint16_t CAMERA_CENTER_ANGLE = 180;
    static constexpr uint16_t CAMERA_MAX_ANGLE = 360;
    static constexpr uint16_t SLIDER_UPDATE_FREQUENCY_HZ = 100;

  public:
    enum class ePlayerState
    {
        NOT_CONNECTED,
        CONNECTING,
        STREAMING,
        RECONNECTING,
        PAUSED,
        CONNECTION_ERROR,
        CONNECTION_FAILED
    };

    QVideoPlayerWidget(std::shared_ptr<rclcpp::Node> guiNode_,
                       std::string url_,
                       uint16_t playerIndex_,
                       std::shared_ptr<QPlayerWorker> workerThreadAruco_,
                       std::shared_ptr<QRecordingWorker> workerThreadRecording_,
                       std::shared_ptr<QPanoramaWorker> workerThreadPanorama_);

    ~QVideoPlayerWidget();

    void setArucoClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_);
    void startDetection(void);
    void stopDetection(void);
    void handleArucoDetection(void);
    void arucoStillAliveUpdate(bool urlFound_);

    void startStream(const QString& rtspUrl_);
    void stopStream(void);
    void setPlayerState(ePlayerState state_);
    void tryReconnect(void);
    void updateStatusText(const QString& text_);
    bool validateRtspUrl(const QString& url_);
    void updateUrlValidationUI(bool isValid_);

    void handlePlayPauseButton(void);

    void setCameraControlClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_);
    void setPanoramaClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::Panorama>> client_);

    std::string getCamURL(void);
    float getCameraAngle(void);
    void setCamURL(std::string newCamUrl_);
    void setURLToDefault(void);
    void updateCamURL(void);

    QTextEdit* getLogWidget()
    {
        return _ui.logDisplay;
    }

    QString getId(void);
    bool isStreaming(void);

    void CB_cameraListUpdate(std::vector<std::string> urls_);
    void CB_srvCameraAvailable(bool available_);
    void CB_updateActualAngle(std::string camURL_, float yaw_);

  signals:
    void arucoCameraFailure(bool valid_);
    void streamStateChanged(bool isRunning_, int streamIndex_);
    void requestStartStream(const QString& rtspUrl_);
    void requestStopStream(void);
    void notifyCameraAnglePublisher(std::string camURL_, float angle_);
    void displayDetectedArucos(std::vector<uint16_t> ids_);

  private slots:
    // Arucuo
    void onDetectionHandledSuccessfully(bool success_, uint16_t playerIndex__);
    void onArucoServerInfoFailed(bool success_);
    void onArucoCameraFailed(bool valid_);
    void onDisplayDetectedArucos(std::vector<uint16_t> ids_);
    // Camera angle
    void onCameraAngleSliderChanged(void);
    void onCameraAngleBoxChanged(void);

    void handlePanorama(void);
    void onPanoramaStarted(uint16_t duration_, uint16_t playerIndex_);
    void onPanoramaFinished(bool success_, const std::string& status_, uint16_t playerIndex_);
    void setPanoramaDuration(void);

    void onPipelineStarted(GstElement* pipeline_);
    void onErrorOccurred(const QString& error_);
    void onConnectionFailed(void);
    void onFrameReceived(void);
    void onFrameTimeout(void);
    void onReconnectTimer(void);
    void onConnectionTimeout(void);
    void onToggleView(void);
    void onUrlTextChanged(const QString& text_);
    void onCenterAngle(void);

    void clearLogs(void);
    void toggleLogView(bool show_);

    void onNewLogMessage(const QString& message_, QWidget* targetWidget_);

  private:
    void setupUI(void);
    void connectUISignals(void);
    void initializeUIState(void);
    void emitStateChanged(void);
    void cleanupResources(void);
    void autoStartGStreamer(void);
    void panoramaTurnCamera(void);

    void hideAngleSelector(void);
    std::shared_ptr<rclcpp::Node> _node;
    Ui::VideoPlayer _ui;

    std::string _camURL = "";
    std::string _defaultCamUrl = "";

    int _streamIndex;
    uint16_t _playerIndex;
    std::string _sessionFolderPath;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> _client_arucoManager;
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::Panorama>> _client_panoramaManager;
    std::shared_ptr<QPlayerWorker> _playerWorkerThreadAruco;
    std::shared_ptr<QRecordingWorker> _playerWorkerThreadRecording;
    std::shared_ptr<QPanoramaWorker> _panoramaWorkerThread;
    QVideoRecorderWidget _recorderWidget;

    ePlayerState _state = ePlayerState::NOT_CONNECTED;
    int _reconnectAttempts = 0;
    bool _wasEverConnected = false;
    bool _controlsVisible = true;
    GstElement* _pipeline = nullptr;
    QThread _gstreamerThread;
    GStreamerWorker* _gstreamerWorker = nullptr;
    QDateTime _lastStreamTime;

    QTimer _reconnectTimer;
    QTimer _frameTimeoutTimer;
    QTimer _connectionTimeoutTimer;
    uint16_t _panoramaDuration = 5000U;
};

#endif