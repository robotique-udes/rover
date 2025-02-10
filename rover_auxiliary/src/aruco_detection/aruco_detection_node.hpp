#ifndef __ARUCO_DETECTION_NODE_HPP__
#define __ARUCO_DETECTION_NODE_HPP__

#include "detection.hpp"
#include "rover_msgs/msg/aruco.hpp"

class ArucoDetectionNode : public rclcpp::Node
{
  public:
    ArucoDetectionNode(void);
    ~ArucoDetectionNode(void);

  private:
    static constexpr bool DEBUG_MODE = true;
    void CB_aruco(void);
    rclcpp::Publisher<rover_msgs::msg::Aruco>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
    std::unique_ptr<Detection> _detection;
};

#endif
