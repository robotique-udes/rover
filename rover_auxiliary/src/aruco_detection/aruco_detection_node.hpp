#ifndef __ARUCO_DETECTION_NODE_HPP__
#define __ARUCO_DETECTION_NODE_HPP__

#include "detection.hpp"
#include "rover_msgs/msg/aruco.hpp"
#include "rover_msgs/srv/aruco_detection.hpp"

class ArucoDetectionNode : public rclcpp::Node
{
    static constexpr uint64_t DELAY_PUBLISHER_MS = 1'000UL;

    // The delay between capture must be paired with the max rate (fps) in the pipeline
    static constexpr uint64_t DELAY_DETECTION_MS = 200UL;
    static constexpr uint8_t ALLOWED_ERROR_FRAME = 50U;

  public:
    ArucoDetectionNode(int argc, char** argv);

  private:
    void getParams(int argc, char** argv);

    void CB_arucoPublisher(void);
    void CB_arucoDetection(void);
    void CB_srv(const std::shared_ptr<rover_msgs::srv::ArucoDetection::Request> request_,
                std::shared_ptr<rover_msgs::srv::ArucoDetection::Response> response_);
    bool startDetection(std::string URL_);
    bool stopDetection(std::string URL_);
    void infoDetection(std::shared_ptr<rover_msgs::srv::ArucoDetection::Response> response_);

    bool _debugMode;
    uint8_t _nbrOngoingDetection = 0;
    std::string _camURL;
    rclcpp::Publisher<rover_msgs::msg::Aruco>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timerPublisher;
    rclcpp::TimerBase::SharedPtr _timerDetection;
    std::unordered_map<std::string, Detection> _detections;
    std::mutex _detectedArucosMutex;
    rclcpp::Service<rover_msgs::srv::ArucoDetection>::SharedPtr _srv_detectionManager;
};

#endif
