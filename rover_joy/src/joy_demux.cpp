#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"
#include "rover_msgs/srv/joy_demux_set_state.hpp"
#include "rovus_lib/macros.h"
#include "rovus_lib/rovus_exceptions.h"

using namespace std::chrono_literals;

class JoyDemux : public rclcpp::Node
{
    enum eControllerType
    {
        main = (int8_t)rover_msgs::srv::JoyDemuxSetState_Request::CONTROLLER_MAIN,
        secondary = (int8_t)rover_msgs::srv::JoyDemuxSetState_Request::CONTROLLER_SECONDARY
    };

    enum eDemuxDestination
    {
        drive_train = (int8_t)rover_msgs::srv::JoyDemuxSetState_Request::DEST_DRIVE_TRAIN,
        arm = (int8_t)rover_msgs::srv::JoyDemuxSetState_Request::DEST_ARM,
        antenna = (int8_t)rover_msgs::srv::JoyDemuxSetState_Request::DEST_ANTENNA,
        none = (int8_t)rover_msgs::srv::JoyDemuxSetState_Request::DEST_NONE
    };

  private:
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_main;
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_secondary;

    rclcpp::Publisher<rover_msgs::msg::Joy>::SharedPtr _pub_drive_train;
    rclcpp::Publisher<rover_msgs::msg::Joy>::SharedPtr _pub_arm;
    rclcpp::Publisher<rover_msgs::msg::Joy>::SharedPtr _pub_antenna;
    rclcpp::Publisher<rover_msgs::msg::JoyDemuxStatus>::SharedPtr _pub_status;

    rclcpp::Service<rover_msgs::srv::JoyDemuxSetState>::SharedPtr _srv_demux;

    rclcpp::TimerBase::SharedPtr _timer_status;

    eDemuxDestination _dest_main = eDemuxDestination::none;
    eDemuxDestination _dest_secondary = eDemuxDestination::none;

    void callbackJoy(const rover_msgs::msg::Joy& msg, int8_t controller_type);
    void callbackDemux(const std::shared_ptr<rover_msgs::srv::JoyDemuxSetState::Request> request,
                       std::shared_ptr<rover_msgs::srv::JoyDemuxSetState::Response> response);
    void callbackStatus();

    void redirectMsg(eDemuxDestination dest, rover_msgs::msg::Joy msg);
    bool isIdle(eDemuxDestination dest);

  public:
    JoyDemux();
    ~JoyDemux() {}
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        rclcpp::spin(std::make_shared<JoyDemux>());
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("Dead Node"), "Killing node on exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}

JoyDemux::JoyDemux(): Node("joy_demux")
{
    _sub_main = this->create_subscription<rover_msgs::msg::Joy>("main_joy",
                                                                1,
                                                                [this](const rover_msgs::msg::Joy msg)
                                                                { callbackJoy(msg, eControllerType::main); });

    _sub_secondary = this->create_subscription<rover_msgs::msg::Joy>("secondary_joy",
                                                                     1,
                                                                     [this](const rover_msgs::msg::Joy msg)
                                                                     { callbackJoy(msg, eControllerType::secondary); });

    _pub_drive_train = this->create_publisher<rover_msgs::msg::Joy>("/rover/drive_train/joy", 1);
    _pub_arm = this->create_publisher<rover_msgs::msg::Joy>("/rover/arm/joy", 1);
    _pub_antenna = this->create_publisher<rover_msgs::msg::Joy>("/base/antenna/joy", 1);
    _pub_status = this->create_publisher<rover_msgs::msg::JoyDemuxStatus>("/joy/demux/status", 1);

    _srv_demux = this->create_service<rover_msgs::srv::JoyDemuxSetState>(
        "demux_control",
        std::bind(&JoyDemux::callbackDemux, this, std::placeholders::_1, std::placeholders::_2));

    _timer_status = this->create_wall_timer(250ms, std::bind(&JoyDemux::callbackStatus, this));
}

void JoyDemux::callbackJoy(const rover_msgs::msg::Joy& msg, int8_t controller_type)
{
    eDemuxDestination dest = eDemuxDestination::none;

    if (controller_type == eControllerType::main)
    {
        dest = _dest_main;
    }
    else if (controller_type == eControllerType::secondary)
    {
        dest = _dest_secondary;
    }
    else
    {
        RCLCPP_WARN(LOGGER, "Wrong \"controller_type\" argument: %i?", controller_type);
    }

    this->redirectMsg(dest, msg);

    // Sending zeros to idling topics but only if main controller callback (to
    // keep frenquency stable)
    if (controller_type == eControllerType::main)
    {
        rover_msgs::msg::Joy msg_zeros;
        if (isIdle(eDemuxDestination::drive_train))
        {
            _pub_drive_train->publish(msg_zeros);
        }

        if (isIdle(eDemuxDestination::arm))
        {
            _pub_arm->publish(msg_zeros);
        }

        if (isIdle(eDemuxDestination::antenna))
        {
            _pub_antenna->publish(msg_zeros);
        }
    }
}

void JoyDemux::callbackStatus()
{
    rover_msgs::msg::JoyDemuxStatus msg_status;
    msg_status.controller_main_topic = _dest_main;
    msg_status.controller_secondary_topic = _dest_secondary;

    _pub_status->publish(msg_status);
}

void JoyDemux::redirectMsg(eDemuxDestination dest, rover_msgs::msg::Joy msg)
{
    if (dest == eDemuxDestination::drive_train)
    {
        _pub_drive_train->publish(msg);
    }
    else if (dest == eDemuxDestination::arm)
    {
        _pub_arm->publish(msg);
    }
    else if (dest == eDemuxDestination::antenna)
    {
        _pub_antenna->publish(msg);
    }

    return;
}

bool JoyDemux::isIdle(eDemuxDestination dest)
{
    return (_dest_main != dest && _dest_secondary != dest);
}

void JoyDemux::callbackDemux(const std::shared_ptr<rover_msgs::srv::JoyDemuxSetState::Request> request,
                             std::shared_ptr<rover_msgs::srv::JoyDemuxSetState::Response> response)
{
    eDemuxDestination dest = (eDemuxDestination)((int8_t)request->destination);

    if (request->controller_type == eControllerType::main)
    {
        if (_dest_secondary == dest)
        {
            RCLCPP_WARN(LOGGER, "Secondary joy topic already redirect to this topic");

            if (request->force)
            {
                RCLCPP_WARN(LOGGER, "Secondary joy destination was set to \"none\"");
                _dest_secondary = eDemuxDestination::none;
            }
            else
            {
                response->success = false;
                return;
            }
        }

        _dest_main = dest;
    }
    else if (request->controller_type == eControllerType::secondary)
    {
        if (_dest_main == dest)
        {
            RCLCPP_WARN(LOGGER, "Main joy topic already redirect to this topic");

            if (request->force)
            {
                RCLCPP_WARN(LOGGER, "Secondary joy can't overwrite main joy");
            }
            response->success = false;
            return;
        }

        _dest_secondary = dest;
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "How did we get here? 0_0");
    }

    response->success = true;
}
