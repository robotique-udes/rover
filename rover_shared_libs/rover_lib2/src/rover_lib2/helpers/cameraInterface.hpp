#ifndef ROVER_LIB2_HELPERS_CAMERAINTERFACE_HPP
#define ROVER_LIB2_HELPERS_CAMERAINTERFACE_HPP

#if defined(ROS)

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/camera_control.hpp"
#include "rover_msgs/msg/camera_config.hpp"
#include "rover_msgs/msg/topic_with_priority.hpp"
#include "rover_lib2/helpers/constants.hpp"

class CameraInterface
{
    static constexpr float SEND_COMMAND_PTZ_FREQUENCY = 5.F;
    static constexpr float SEND_CONFIG_PTZ_FREQUENCY = 0.5F;
    static constexpr float SEND_COMMAND_POWER_FREQUENCY = 0.5F;

    static constexpr float RECEIVE_PTZ_STATUS_FREQUENCY = 0.5F;
    static constexpr float RECEIVE_POWER_STATUS_FREQUENCY = 0.5F;

    static constexpr const char* POWER_STATUS_TOPIC = "/rover/camera/power_status";
    static constexpr const char* PTZ_STATUS_TOPIC = "/rover/camera/PTZ_status";
    static constexpr const char* TOPIC_WITH_PRIORITY = "/rover/camera/topic_with_priority";

    static constexpr float EPSILON = 0.1;

  public:
    CameraInterface(std::shared_ptr<rclcpp::Node> node_,
                    const std::string& ptzCommandTopic_,
                    const std::string& ptzConfigTopic_,
                    const std::string& powerCommandTopic_);

    void setPTZCmd(rover_msgs::msg::CameraControl goalMsg_, size_t id_);
    rover_msgs::msg::CameraControl getPTZCmd(size_t id_) const;

    void setPTZConfig(rover_msgs::msg::CameraConfig configMsg_, size_t id_);
    rover_msgs::msg::CameraConfig getPtzConfig(size_t id_) const;

    void setPowerCmd(rover_msgs::msg::CameraControl powerMsg_, size_t id_);
    rover_msgs::msg::CameraControl getPowerCmd(size_t id_) const;

    void release(size_t id_);

    rover_msgs::msg::CameraControl getLastPowerStatusMsg(size_t id_) const;
    rover_msgs::msg::CameraControl getLastPtzStatusMsg(size_t id_) const;

    bool isGoalReached(size_t id_);
    bool isCamUnderControl(size_t id_);

  private:
    void initTimers();
    void initSub();
    void initPub();

    void CB_publishPtzCmd(void);
    void CB_publishPtzConfig(void);
    void CB_publishPowerCmd(void);

    void CB_subscriberPowerStatus(rover_msgs::msg::CameraControl statusMsg_);
    void CB_subscriberPtzStatus(rover_msgs::msg::CameraControl statusMsg_);
    void CB_subscriberTopicWithPriority(rover_msgs::msg::TopicWithPriority topicLists_);

    std::string _ptzCommandTopic;
    std::string _ptzConfigTopic;
    std::string _powerCommandTopic;

    rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _pub_PTZCmd;
    rclcpp::Publisher<rover_msgs::msg::CameraConfig>::SharedPtr _pub_configCmd;
    rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _pub_powerCmd;
    rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr _sub_powerStatus;
    rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr _sub_PTZStatus;
    rclcpp::Subscription<rover_msgs::msg::TopicWithPriority>::SharedPtr _sub_topicWithPriority;

    rclcpp::TimerBase::SharedPtr _timer_pubPTZCmd;
    rclcpp::TimerBase::SharedPtr _timer_pubPTZConfig;
    rclcpp::TimerBase::SharedPtr _timer_pubPowerCmd;

    std::array<rover_msgs::msg::CameraControl, static_cast<std::size_t>(Constants::CameraInfo::eCamNames::eLast)> _lastPtzCmdMsg;
    std::array<rover_msgs::msg::CameraConfig, static_cast<std::size_t>(Constants::CameraInfo::eCamNames::eLast)>
        _lastPtzConfigMsg;
    std::array<rover_msgs::msg::CameraControl, static_cast<std::size_t>(Constants::CameraInfo::eCamNames::eLast)> _lastPowerMsg;

    std::array<rover_msgs::msg::CameraControl, static_cast<std::size_t>(Constants::CameraInfo::eCamNames::eLast)>
        _lastPowerStatusMsg;
    std::array<rover_msgs::msg::CameraControl, static_cast<std::size_t>(Constants::CameraInfo::eCamNames::eLast)>
        _lastPtzStatusMsg;

    std::array<std::string, static_cast<std::size_t>(Constants::CameraInfo::eCamNames::eLast)> _topicWithPriority;

    std::array<bool, static_cast<std::size_t>(Constants::CameraInfo::eCamNames::eLast)> _isCamConcerned = {false};
    std::array<bool, static_cast<std::size_t>(Constants::CameraInfo::eCamNames::eLast)> _isGoalReached = {false};

    std::shared_ptr<rclcpp::Node> _node;
};

#endif  // defined (ROS)
#endif  // ROVER_LIB2_HELPERS_CAMERAINTERFACE_HPP
