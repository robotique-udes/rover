#ifndef QVIDEOPLAYERWIDGER_HPP
#define QVIDEOPLAYERWIDGER_HPP

#include "rclcpp/rclcpp.hpp"

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <QTimer>
#include "UI_VideoPlayer.h"
#include "Worker/QPlayerWorker.hpp"
#include <Global/Helpers/QToastNotification/QToastNotification.hpp>

class QVideoPlayerWidget : public QWidget
{
    Q_OBJECT

    static constexpr size_t DELAY_OPENING_CAM_RETRY_MS = 5'000UL;
    static constexpr size_t MAX_DELAY_SERVICE_CALL = 2'000UL;
    static constexpr size_t NBR_IDS_TO_DISPLAY = 5U;
    static constexpr size_t STYLE_RESET_TIME = 2'000UL;

  public:
    QVideoPlayerWidget(std::shared_ptr<rclcpp::Node> guiNode_,
                       std::string url_,
                       uint16_t tag_,
                       std::shared_ptr<QPlayerWorker> worker_,
                       std::shared_ptr<QPlayerWorker> worker2_);

    void setArucoClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_);

    void startDetection(void);
    void stopDetection(void);
    void handleArucoDetection(void);
    void arucoStillAliveUpdate(bool urlFound_);
    void displayDetectedArucos(std::vector<uint16_t> ids_);

    void handlePlayPauseButton(void);

    void setCameraControlClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_);

    std::string getCamURL(void);
    void setCamURL(std::string newCamUrl_);
    void setURLToDefault(void);
    void updateCamURL(void);

    void CB_cameraListUpdate(std::vector<std::string> urls_);

  signals:
    void arucoCameraFailure(bool valid_);

  private slots:
  //Arucuo
    void onDetectionHandledSuccessfully(bool success_, uint16_t tag_);
    void onArucoServerInfoFailed(bool success_);
    void onArucoCameraFailed(bool valid_);
  //Camera server
    void handleScreenshot(void);
    void handleRecording(void);
    void onScreenshotHandledSuccessfully(bool success_, std::string status_, uint16_t tag_);
    void onStartRecordingHandledSuccessfully(bool success_, std::string status_, uint16_t tag_);
    void onStopRecordingHandledSuccessfully(bool success_, std::string status_, uint16_t tag_);

  private:
    std::shared_ptr<rclcpp::Node> _node;
    Ui::VideoPlayer _ui;

    std::string _camURL = "";
    std::string _defaultCamUrl = "";
    uint16_t _tag;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> _client_arucoManager;
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> _client_cameraControlManager;

    std::shared_ptr<QPlayerWorker> _playerWorkerThread;
    std::shared_ptr<QPlayerWorker> _playerWorkerThread2;
};

#endif  // QVIDEOPLAYERWIDGER_HPP
