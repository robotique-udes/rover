#ifndef __VIDEO_WORKER_HPP__
#define __VIDEO_WORKER_HPP__

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

#include <QObject>
#include <QProgressBar>
#include <QString>

#include "Global/Workers/QWorker.hpp"
#include "rover_msgs/srv/aruco_detection.hpp"
#include "rclcpp/rclcpp.hpp"




class QPlayerWorker : public QWorker
{
    Q_OBJECT

  public:
    QPlayerWorker(bool start_ = false, QObject* parent_ = nullptr);
    ~QPlayerWorker();
    void manageDetection(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> 
    client_ArucoDetectionManager_, std::string _camURL, bool start_);
    void manageDetectionInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> 
    client_ArucoDetectionManager_, std::string _camURL, bool start_);

    void updateDetectionManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager_,std::string camURL_);
    void updateDetectionInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_ArucoDetectionManager_,std::string camURL_);


  signals:
    void detectionHandledSuccessfully(bool success);
  signals:
    void urlFoundInDetection(bool url_found);

  private:

      
};

#endif  // __VIDEO_WORKER_HPP__
