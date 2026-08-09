#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_options.hpp>
#include <chrono>

#include <rover_msgs/msg/joy.hpp>
#include <rover_msgs/msg/joy_demux_status.hpp>
#include <rover_msgs/srv/joy_demux_set_state.hpp>
#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>

class JoyDemux : public rclcpp::Node
{
  private:
    enum class eControllerType : uint8_t
    {
        MAIN = rover_msgs::srv::JoyDemuxSetState_Request::CONTROLLER_MAIN,
        SECONDARY = rover_msgs::srv::JoyDemuxSetState_Request::CONTROLLER_SECONDARY
    };

    enum class eDemuxDestination : uint8_t
    {
        DRIVE_TRAIN = rover_msgs::srv::JoyDemuxSetState_Request::DEST_DRIVE_TRAIN,
        ARM = rover_msgs::srv::JoyDemuxSetState_Request::DEST_ARM,
        ANTENNA = rover_msgs::srv::JoyDemuxSetState_Request::DEST_ANTENNA,
        NONE = rover_msgs::srv::JoyDemuxSetState_Request::DEST_NONE
    };

    static constexpr std::chrono::milliseconds TELEOP_DEADLINE = std::chrono::milliseconds(200);
    static constexpr std::chrono::milliseconds TELEOP_LEASE_DURATION = std::chrono::milliseconds(300);

  public:
    JoyDemux();

  private:
    void CB_joy(const rover_msgs::msg::Joy& msg_, eControllerType controller_type_) const;
    void CB_demux(const std::shared_ptr<rover_msgs::srv::JoyDemuxSetState::Request> request_,
                  std::shared_ptr<rover_msgs::srv::JoyDemuxSetState::Response> response_);
    void CB_status() const;

    void redirectMsg(eDemuxDestination dest_, const rover_msgs::msg::Joy& msg_) const;
    bool isIdle(eDemuxDestination dest_) const;

    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_main;
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_secondary;

    rclcpp::Publisher<rover_msgs::msg::Joy>::SharedPtr _pub_drive_train;
    rclcpp::Publisher<rover_msgs::msg::Joy>::SharedPtr _pub_arm;
    rclcpp::Publisher<rover_msgs::msg::Joy>::SharedPtr _pub_antenna;
    rclcpp::Publisher<rover_msgs::msg::JoyDemuxStatus>::SharedPtr _pub_status;

    rclcpp::Service<rover_msgs::srv::JoyDemuxSetState>::SharedPtr _srv_demux;

    rclcpp::TimerBase::SharedPtr _timer_status;

    eDemuxDestination _dest_main = eDemuxDestination::NONE;
    eDemuxDestination _dest_secondary = eDemuxDestination::DRIVE_TRAIN;
};

int main(int argc_, char* argv_[])
{
    rclcpp::init(argc_, argv_);

    rclcpp::spin(std::make_shared<JoyDemux>());

    rclcpp::shutdown();
    return 0;
}

JoyDemux::JoyDemux():
    Node("joy_demux")
{
    _sub_main = this->create_subscription<rover_msgs::msg::Joy>("main_joy",
                                                                QOS_DEFAULT,
                                                                [this](const rover_msgs::msg::Joy& msg_)
                                                                {
                                                                    CB_joy(msg_, eControllerType::MAIN);
                                                                });

    _sub_secondary = this->create_subscription<rover_msgs::msg::Joy>("secondary_joy",
                                                                     QOS_DEFAULT,
                                                                     [this](const rover_msgs::msg::Joy& msg_)
                                                                     {
                                                                         CB_joy(msg_, eControllerType::SECONDARY);
                                                                     });

    _pub_drive_train = this->create_publisher<rover_msgs::msg::Joy>("drive_train", QOS_DEFAULT);

    rclcpp::QoS teleopQos(rclcpp::KeepLast(1));
    teleopQos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    teleopQos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    teleopQos.deadline(TELEOP_DEADLINE);
    teleopQos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
    teleopQos.liveliness_lease_duration(TELEOP_LEASE_DURATION);

    _pub_arm = this->create_publisher<rover_msgs::msg::Joy>("arm", teleopQos);

    _pub_antenna = this->create_publisher<rover_msgs::msg::Joy>("antenna", QOS_DEFAULT);
    _pub_status = this->create_publisher<rover_msgs::msg::JoyDemuxStatus>("demux_status", QOS_DEFAULT);

    _srv_demux = this->create_service<rover_msgs::srv::JoyDemuxSetState>(
        "demux_control",
        [this](const std::shared_ptr<rover_msgs::srv::JoyDemuxSetState::Request> request_,
               std::shared_ptr<rover_msgs::srv::JoyDemuxSetState::Response> response_)
        {
            this->CB_demux(request_, response_);
        });

    _timer_status = this->create_wall_timer(std::chrono::milliseconds(250),
                                            [this]()
                                            {
                                                this->CB_status();
                                            });
}

void JoyDemux::CB_joy(const rover_msgs::msg::Joy& msg_, eControllerType controller_type_) const
{
    eDemuxDestination dest = eDemuxDestination::NONE;

    if (controller_type_ == eControllerType::MAIN)
    {
        dest = _dest_main;
    }
    else if (controller_type_ == eControllerType::SECONDARY)
    {
        dest = _dest_secondary;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Wrong \"controller_type\" argument: %u?", std::to_underlying(controller_type_));
    }

    this->redirectMsg(dest, msg_);

    // Sending zeros to idling topics but only if main controller callback (to
    // keep frenquency stable)
    if (controller_type_ == eControllerType::MAIN)
    {
        rover_msgs::msg::Joy msg_zeros;
        if (isIdle(eDemuxDestination::DRIVE_TRAIN))
        {
            _pub_drive_train->publish(msg_zeros);
        }

        if (isIdle(eDemuxDestination::ARM))
        {
            _pub_arm->publish(msg_zeros);
        }

        if (isIdle(eDemuxDestination::ANTENNA))
        {
            _pub_antenna->publish(msg_zeros);
        }
    }
}

void JoyDemux::CB_status() const
{
    rover_msgs::msg::JoyDemuxStatus msg_status;
    msg_status.controller_main_topic = std::to_underlying(_dest_main);
    msg_status.controller_secondary_topic = std::to_underlying(_dest_secondary);

    _pub_status->publish(msg_status);
}

void JoyDemux::redirectMsg(eDemuxDestination dest_, const rover_msgs::msg::Joy& msg_) const
{
    if (dest_ == eDemuxDestination::DRIVE_TRAIN)
    {
        _pub_drive_train->publish(msg_);
    }
    else if (dest_ == eDemuxDestination::ARM)
    {
        _pub_arm->publish(msg_);
    }
    else if (dest_ == eDemuxDestination::ANTENNA)
    {
        _pub_antenna->publish(msg_);
    }

    return;
}

bool JoyDemux::isIdle(eDemuxDestination dest_) const
{
    return (_dest_main != dest_ && _dest_secondary != dest_);
}

void JoyDemux::CB_demux(const std::shared_ptr<rover_msgs::srv::JoyDemuxSetState::Request> request,
                        std::shared_ptr<rover_msgs::srv::JoyDemuxSetState::Response> response)
{
    eDemuxDestination dest = (eDemuxDestination)((int8_t)request->destination);

    if (request->controller_type == std::to_underlying(eControllerType::MAIN))
    {
        if (_dest_secondary == dest)
        {
            RCLCPP_WARN(this->get_logger(), "Secondary joy topic already redirect to this topic");

            if (request->force)
            {
                RCLCPP_WARN(this->get_logger(), "Secondary joy destination was set to \"none\"");
                _dest_secondary = eDemuxDestination::NONE;
            }
            else
            {
                response->success = false;
                return;
            }
        }

        _dest_main = dest;
    }
    else if (request->controller_type == std::to_underlying(eControllerType::SECONDARY))
    {
        if (_dest_main == dest)
        {
            RCLCPP_WARN(this->get_logger(), "Main joy topic already redirect to this topic");

            if (request->force)
            {
                RCLCPP_WARN(this->get_logger(), "Secondary joy can't overwrite main joy");
            }
            response->success = false;
            return;
        }

        _dest_secondary = dest;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "How did we get here? 0_0");
    }

    response->success = true;
}
