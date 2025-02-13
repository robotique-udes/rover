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
    static constexpr uint16_t DELAY_PUBLISHER_MS = 1000;
    static constexpr uint16_t DELAY_DETECTION_MS = 50;
    void CB_aruco_publisher(void);
    void CB_aruco_detection(void);
    rclcpp::Publisher<rover_msgs::msg::Aruco>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timerPublisher;
    rclcpp::TimerBase::SharedPtr _timerDetection;
    std::unique_ptr<Detection> _detection;
    std::mutex _mutex;
};

#endif
