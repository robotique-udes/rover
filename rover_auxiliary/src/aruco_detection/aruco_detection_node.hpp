#ifndef __ARUCO_DETECTION_NODE_HPP__
#define __ARUCO_DETECTION_NODE_HPP__

#include "detection.hpp"
#include "rover_msgs/msg/aruco.hpp"

class ArucoDetectionNode : public rclcpp::Node
{
  public:
    ArucoDetectionNode(void);

  private:
    bool DEBUG_MODE;
    void CB_aruco_publisher(void);
    void CB_aruco_detection(void);
    rclcpp::Publisher<rover_msgs::msg::Aruco>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer_publisher;
    rclcpp::TimerBase::SharedPtr _timer_detection;
    std::unique_ptr<Detection> _detection;
    std::mutex _mutex;
};

#endif
