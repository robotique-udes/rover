#ifndef __VIDEO_WORKER_HPP__
#define __VIDEO_WORKER_HPP__

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

#include <QObject>
#include <QString>

#include "Global/Workers/QWorker.hpp"
#include "rover_msgs/srv/aruco_detection.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rovus_lib/timer.hpp"

class QPlayerWorker : public QWorker
{
    Q_OBJECT

    static constexpr uint64_t MAX_DELAY_SERVICE_CALL = 2000UL;

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

  signals:
    void detectionHandledSuccessfully(bool success_, uint16_t tag_);
    void urlFoundInDetection(std::vector<std::string> urls_found);
    void arucoServerInfoFailed(bool success);

  private:
    RoverLib::Timer<uint64_t, RoverLib::millis> _timer_serviceCall;
};

#endif  // __VIDEO_WORKER_HPP__
