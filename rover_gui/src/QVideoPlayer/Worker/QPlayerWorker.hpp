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

  // static constexp

  public:
    QPlayerWorker(bool start_ = false, QObject* parent_ = nullptr);
    ~QPlayerWorker();
    void startDetection(std::shared_ptr<rclcpp::Node> node_, std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> 
    client_ArucoDetectionManager_, std::string _camURL);


signals:
    void detectionStartedSuccessfully(bool success);

  private:
      
};

#endif  // __VIDEO_WORKER_HPP__
