#ifndef __ARUCO_DETECTION_NODE_HPP__
#define __ARUCO_DETECTION_NODE_HPP__

#include "detection.hpp"
#include "rover_msgs/msg/aruco.hpp"

class ArucoDetectionNode : public rclcpp::Node
{
    static constexpr uint64_t DELAY_PUBLISHER_MS = 1'000UL;
    static constexpr uint64_t DELAY_DETECTION_MS = 50UL;

  public:
    ArucoDetectionNode(int argc, char** argv);

  private:
    void getParams(int argc, char** argv);

    void CB_arucoPublisher(void);
    void CB_arucoDetection(void);

    bool _debugMode;
    std::string _camURL;
    rclcpp::Publisher<rover_msgs::msg::Aruco>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timerPublisher;
    rclcpp::TimerBase::SharedPtr _timerDetection;
    std::unique_ptr<Detection> _detection;
    std::mutex _detectedArucosMutex;
};

#endif
