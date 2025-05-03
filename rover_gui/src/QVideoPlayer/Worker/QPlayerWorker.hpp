#ifndef VIDEO_WORKER_HPP
#define VIDEO_WORKER_HPP

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

#include <QObject>
#include <QString>

#include "Global/Workers/QWorker.hpp"
#include "rover_msgs/srv/aruco_detection.hpp"
#include "rover_msgs/srv/camera_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rovus_lib/timer.hpp"

class QPlayerWorker : public QWorker
{
    Q_OBJECT

    static constexpr uint64_t MAX_DELAY_SERVICE_CALL = 2'000UL;

  public:
    QPlayerWorker(bool start_ = false, QObject* parent_ = nullptr);
    ~QPlayerWorker();

    void manageDetection(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager_,
                         std::string _camURL,
                         uint16_t tag_,
                         bool start_);
    void manageDetectionInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager_,
                                 std::string _camURL,
                                 uint16_t tag_,
                                 bool start_);

    void updateDetectionManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager_);
    void updateDetectionInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager_);

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
    void detectionHandledSuccessfully(bool success_, uint16_t tag_);
    void urlFoundInDetection(std::vector<std::string> urls_found);
    void arucoServerInfoFailed(bool success);
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

    RoverLib::Timer<uint64_t, RoverLib::millis> _timer_serviceCall;
    RoverLib::Timer<uint64_t, RoverLib::millis> _timer_serviceCallCamera;
};

#endif  // VIDEO_WORKER_HPP
