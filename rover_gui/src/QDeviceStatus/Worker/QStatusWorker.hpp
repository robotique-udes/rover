#ifndef __QSTATUS_WORKER_HPP__
#define __QSTATUS_WORKER_HPP__

#include "Global/Workers/QWorker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/srv/empty.hpp"
#include "rover_lib2/helpers/macros.hpp"

class QStatusWorker : public QWorker
{
    Q_OBJECT

  public:
    QStatusWorker(bool start_ = false, QObject* parent_ = nullptr);
    ~QStatusWorker();

    void requestDeviceStatusManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::Empty>> client_requestErrorStatus_);
    void requestDeviceStatusInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::Empty>> client_requestErrorStatus_);

  signals:
    void onRequestDeviceStatusSuccessful(bool success_, const std::string status_);
};

#endif  // __QSTATUS_WORKER_HPP__