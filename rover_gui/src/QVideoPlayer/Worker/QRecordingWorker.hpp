#ifndef RECORDING_WORKER_HPP
#define RECORDING_WORKER_HPP

#include <QObject>
#include <QString>

#include "Global/Workers/QWorker.hpp"
#include "rover_msgs/srv/camera_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_lib2/helpers/time.hpp"

class QRecordingWorker : public QWorker
{
    Q_OBJECT

    static constexpr uint64_t MAX_DELAY_SERVICE_CALL = 4'000UL;
    static constexpr uint16_t SERVICE_POLL_INTERVAL = 100U;

  public:
    QRecordingWorker(bool start_ = false, QObject* parent_ = nullptr);
    ~QRecordingWorker();

    void takeScreenshotManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                               std::string cameraUrl_,
                               uint16_t playerIndex_,
                               std::string basePath_);

    void startRecordingManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                               std::string cameraUrl_,
                               uint16_t playerIndex_,
                               std::string basePath_);

    void stopRecordingManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                              std::string cameraUrl_,
                              uint16_t playerIndex_,
                              std::string basePath_);

  signals:
    void screenshotHandledSuccessfully(bool success_, std::string status_, uint16_t playerIndex_);
    void startRecordingHandledSuccessfully(bool success, std::string status, uint16_t playerIndex_);
    void stopRecordingHandledSuccessfully(bool success, std::string status, uint16_t playerIndex_);
    void setCursorWaiting(bool waiting_);

  private:
    void takeScreenshotInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                                std::string cameraUrl_,
                                uint16_t playerIndex_,
                                std::string basePath_);

    void startRecordingInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                                std::string camera_URL_,
                                uint16_t playerIndex_,
                                std::string basePath_);

    void stopRecordingInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                               std::string cameraUrl_,
                               uint16_t playerIndex_,
                               std::string basePath_);

    LoopTimer<uint64_t, Time::millis> _timer_serviceCallCamera;
};
#endif  // RECORDING_WORKER_HPP