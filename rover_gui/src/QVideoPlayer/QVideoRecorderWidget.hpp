#ifndef QVIDEOPLAYER_QVIDEORECORDERWIDGET_HPP
#define QVIDEOPLAYER_QVIDEORECORDERWIDGET_HPP

#include "rclcpp/rclcpp.hpp"

#include "rover_msgs/srv/camera_control.hpp"
#include "Worker/QRecordingWorker.hpp"
#include "UI_VideoPlayer.h"
#include <QtWidgets/QWidget>

struct sRecordingButtons
{
    QPushButton* startRecordingButton;
    QPushButton* screenshotButton;
};

class QVideoRecorderWidget : public QWidget
{
    Q_OBJECT

    static constexpr size_t STYLE_RESET_TIME = 2'000UL;

  public:
    QVideoRecorderWidget(sRecordingButtons buttons_,
                         std::string url_,
                         uint16_t playerIndex__,
                         std::shared_ptr<QRecordingWorker> workerThreadRecording_);

    ~QVideoRecorderWidget();

    void updateCamURL(std::string& url_);
    void setCameraControlClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_);

    void CB_cameraListUpdate(std::vector<std::string> urls_);
    void CB_serviceCameraControlAvailable(bool available_);

  private slots:
    void handleScreenshot(void);
    void handleRecording(void);
    void onScreenshotHandledSuccessfully(bool success_, std::string status_, uint16_t tag_);
    void onStartRecordingHandledSuccessfully(bool success_, std::string status_, uint16_t tag_);
    void onStopRecordingHandledSuccessfully(bool success_, std::string status_, uint16_t tag_);

  private:
    int16_t _playerIndex;
    std::string _camURL = "";
    std::string _sessionFolderPath;

    QPushButton* _startRecordingButton;
    QPushButton* _screenshotButton;
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> _client_cameraControlManager;
    std::shared_ptr<QRecordingWorker> _playerWorkerThreadRecording;
};

#endif  // QVIDEOPLAYER_QVIDEORECORDERWIDGET_HPP