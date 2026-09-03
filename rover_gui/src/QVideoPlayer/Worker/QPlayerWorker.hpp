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
#include "rover_msgs/srv/camera_ir.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_lib2/helpers/time.hpp"

class QPlayerWorker : public QWorker
{
    Q_OBJECT

    static constexpr uint64_t MAX_DELAY_SERVICE_CALL = 4'000UL;
    static constexpr uint16_t SERVICE_POLL_INTERVAL = 100U;

  public:
    QPlayerWorker(bool start_ = false, QObject* parent_ = nullptr);
    ~QPlayerWorker();

    void manageDetection(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager_,
                         std::string _camURL,
                         uint16_t playerIndex_,
                         bool start_);
    void manageDetectionInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager_,
                                 std::string _camURL,
                                 uint16_t playerIndex_,
                                 bool start_);

    void updateDetectionManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager_);
    void updateDetectionInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager_);
    void toggleIRMode(rclcpp::Client<rover_msgs::srv::CameraIR>::SharedPtr client_cameraIR_,
                      const std::string& ip_,
                      uint8_t mode_);

  signals:
    void detectionHandledSuccessfully(bool success_, uint16_t playerIndex_);
    void urlFoundInDetection(std::vector<std::string> urls_found);
    void arucoServerInfoFailed(bool success);

  private:
    void toggleIRModeInternal(rclcpp::Client<rover_msgs::srv::CameraIR>::SharedPtr client_cameraIR_,
                              const std::string& ip_,
                              uint8_t mode_);

    LoopTimer<uint64_t, Time::millis> _timer_serviceCall;
};

#endif
