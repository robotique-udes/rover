#ifndef QVIDEOPLAYER_QVIDEORECORDERWIDGET_HPP
#define QVIDEOPLAYER_QVIDEORECORDERWIDGET_HPP

#include "Worker/QRecordingWorker.hpp"
#include <QtWidgets/QWidget>
#include <QtWidgets/QPushButton>

struct sRecordingButtons
{
    QPushButton* startRecordingButton = nullptr;
    QPushButton* screenshotButton = nullptr;
};

class QVideoRecorderWidget : public QWidget
{
    Q_OBJECT

    static constexpr size_t STYLE_RESET_TIME = 2'000UL;

  public:
    QVideoRecorderWidget(const std::string& url_,
                         uint16_t playerIndex__,
                         std::shared_ptr<QRecordingWorker> workerThreadRecording_);

    void updateCamURL(const std::string& url_);
    void setButtons(const sRecordingButtons& buttons_);
    void setCameraControlClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_);

    void CB_srvAvailable(bool available_);
    void emitUpdateCameraList(const std::vector<std::string>& urls);
  signals:
    void updateCameraList(const std::vector<std::string>& urls);

  private slots:
    void handleScreenshot(void) const;
    void handleRecording(void) const;
    void onScreenshotHandledSuccessfully(bool success_, const std::string& status_, uint16_t playerIndex_);
    void onStartRecordingHandledSuccessfully(bool success_, const std::string& status_, uint16_t playerIndex_);
    void onStopRecordingHandledSuccessfully(bool success_, const std::string& status_, uint16_t playerIndex_);
    void onUpdateCameraList(const std::vector<std::string>& urls_);

  private:
    void autoStartRecording(void);

    uint16_t _playerIndex;
    std::string _camURL;
    std::string _sessionFolderPath;

    sRecordingButtons _sButtons;
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> _client_cameraControlManager;
    std::shared_ptr<QRecordingWorker> _playerWorkerThreadRecording;
};

#endif  // QVIDEOPLAYER_QVIDEORECORDERWIDGET_HPP