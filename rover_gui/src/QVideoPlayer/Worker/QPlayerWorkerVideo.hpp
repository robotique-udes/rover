#ifndef RECORDING_WORKER_HPP
#define RECORDING_WORKER_HPP

#include "Global/Workers/QWorker.hpp"
#include "rover_msgs/srv/camera_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_lib2/helpers/time.hpp"

class QPlayerWorkerVideo : public QWorker
{
    Q_OBJECT

    static constexpr uint64_t MAX_DELAY_SERVICE_CALL = 2'000UL;

  public:
    QPlayerWorkerVideo(bool start_ = false, QObject* parent_ = nullptr);
    ~QPlayerWorkerVideo();

    void takeScreenshotManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                               std::string camera_URL_,
                               uint16_t tag_);

    void startRecordingManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                               std::string camera_URL_,
                               uint16_t tag_);

    void stopRecordingManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                              std::string camera_URL_,
                              uint16_t tag_);
  signals:
    void screenshotHandledSuccessfully(bool success_, std::string status_, uint16_t tag_);
    void startRecordingHandledSuccessfully(bool success, std::string status, uint16_t tag_);
    void stopRecordingHandledSuccessfully(bool success, std::string status, uint16_t tag_);

  private:
    void takeScreenshotInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                                std::string camera_URL_,
                                uint16_t tag_);

    void startRecordingInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                                std::string camera_URL_,
                                uint16_t tag_);

    void stopRecordingInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_CameraControl_,
                               std::string camera_URL_,
                               uint16_t tag_);

    LoopTimer<uint64_t, Time::millis> _timer_serviceCallCamera;
};

#endif  // RECORDING_WORKER_HPP