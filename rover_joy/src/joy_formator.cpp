#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rovus_lib/rovus_exceptions.h"
#include "rovus_lib/macros.h"

// =============================================================================
// This node subscribe to a topic of message type "sensor_msgs/msg/joy" and
// remaps the msg to a more readable/"humain friendly" custom ros msg
// "rover_msgs/msg/joy". The keybinding will correspond to the specified
// controller in parameter.
//
// When running this node you MUST set the "controller_type" parameter to a
// valid controller (see the JoyFormator::setControllerType() method definition)
//
// New controllers can be added inside JoyFormator::setControllerType() method.
//
// This node sends msgs at a fixed rate and will sends zeros if no input have
// been received before a specified timeout. This is not a watchdog, it just
// handles controller deconnections
// =============================================================================

using namespace std::chrono_literals;

class Keybinding
{
public:
    // Enum MUST start at 0 since they are used as array indexes
    enum eKeybinding
    {
        a = 0,
        b,
        x,
        y,
        l1,
        r1,
        joystick_left_push,
        joystick_right_push,
        ext0,
        ext1,
        ext2,
        joystick_left_side,
        joystick_left_front,
        l2,
        joystick_right_side,
        joystick_right_front,
        r2,
        cross_front,
        cross_side,
        eKeybinding_END
    };
};

class JoyFormator : public rclcpp::Node
{
public:
    struct sControllerConfig
    {
        int8_t buttons[Keybinding::eKeybinding_END] = {0};
        int8_t axes[Keybinding::eKeybinding_END] = {0};

        float trigger_range_min = -1.0f;
        float trigger_range_max = 1.0f;

        float joystick_dead_zone = 0.0f;

        // Point this pointer to a function for a controller which needs
        // specific custom execution each publish loop. This can be used to
        // handle a weird deconnection from controller
        void (JoyFormator::*custom_steps)(rover_msgs::msg::Joy *formatted_joy);

        sControllerConfig()
        {
            custom_steps = NULL;
            FOR_ALL(buttons)
            {
                buttons[i] = -1;
            }

            FOR_ALL(axes)
            {
                axes[i] = -1;
            }
        }
    };

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _sub_joy;
    rclcpp::Publisher<rover_msgs::msg::Joy>::SharedPtr _pub_joy_formatted;
    rclcpp::TimerBase::SharedPtr _timer;

    sControllerConfig _controller_config;

    sensor_msgs::msg::Joy _current_joy;
    sensor_msgs::msg::Joy _last_joy;
    rover_msgs::msg::Joy _last_formatted_joy_msg;
    rclcpp::Time _last_joy_time = this->now();
    rclcpp::Duration _timeout = rclcpp::Duration(1.0f, 0U);

    bool _controller_detected = false;
    bool _controller_reset_needed = false;

    void callbackJoy(const sensor_msgs::msg::Joy &msg);
    void callbackPubJoy();
    void setControllerType(std::string controller_type_name);
    template <typename T>
    T getJoyValue(Keybinding::eKeybinding key);
    float applyJoystickDeadZone(float value_);
    bool connected();
    void executeCustomSteps(rover_msgs::msg::Joy *formatted_joy);
    void customStepsLogitech(rover_msgs::msg::Joy *formatted_joy);

public:
    JoyFormator();
    ~JoyFormator() {}
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<JoyFormator>());
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("Dead Node"), "Killing node on exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}

JoyFormator::JoyFormator() : Node("joy_formator")
{
    this->declare_parameter<std::string>("controller_type", "not_set");
    this->declare_parameter<int16_t>("disconnect_timeout_ms", 1000);

    rclcpp::Parameter param_controller_type = this->get_parameter("controller_type");
    this->setControllerType(param_controller_type.as_string());
    rclcpp::Parameter param_timeout = this->get_parameter("disconnect_timeout_ms");
    _timeout = rclcpp::Duration((float)param_timeout.as_int() / 1000.0f, 0U);

    _sub_joy = this->create_subscription<sensor_msgs::msg::Joy>("raw/joy", 1, std::bind(&JoyFormator::callbackJoy, this, std::placeholders::_1));
    _pub_joy_formatted = this->create_publisher<rover_msgs::msg::Joy>("formated/joy", 1);

    _timer = this->create_wall_timer(10ms, std::bind(&JoyFormator::callbackPubJoy, this));
}

void JoyFormator::callbackJoy(const sensor_msgs::msg::Joy &msg)
{
    if (!_controller_detected)
    {
        _controller_detected = true;
    }
    _last_joy_time = this->now();
    _current_joy = msg;
}

void JoyFormator::callbackPubJoy()
{
    rover_msgs::msg::Joy formatted_joy_msg;

    // Sending zeros if no controller msgs have ever been received but doesn't 
    // acts as if it was deconnected
    if (!_controller_detected)
    {
        executeCustomSteps(&formatted_joy_msg);
        _pub_joy_formatted->publish(formatted_joy_msg);
        _last_formatted_joy_msg = formatted_joy_msg;
        return;
    }

    // Sending zeros as safety if no new msg received in timeout
    if (!connected())
    {
        RCLCPP_WARN_THROTTLE(LOGGER, CLOCK, 5000, "Controller deconnected!");

        executeCustomSteps(&formatted_joy_msg);
        _pub_joy_formatted->publish(formatted_joy_msg);
        _last_formatted_joy_msg = formatted_joy_msg;
        return;
    }

    // When no new message, sending last formatted_joy msg and skip publish loop
    // until first joy msg is received otherwise it will overflows joy's
    // std::vectors
    if ((_current_joy == _last_joy) || (_current_joy.buttons.size() == 0 || _current_joy.axes.size() == 0))
    {
        executeCustomSteps(&formatted_joy_msg);
        _pub_joy_formatted->publish(_last_formatted_joy_msg);
        return;
    }
    
    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::A] = getJoyValue<bool>(Keybinding::a);
    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::B] = getJoyValue<bool>(Keybinding::b);
    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::X] = getJoyValue<bool>(Keybinding::x);
    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::Y] = getJoyValue<bool>(Keybinding::y);
    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::L1] = getJoyValue<bool>(Keybinding::l1);
    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::R1] = getJoyValue<bool>(Keybinding::r1);
    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_PUSH] = getJoyValue<bool>(Keybinding::joystick_left_push);
    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_PUSH] = getJoyValue<bool>(Keybinding::joystick_right_push);
    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::EXT0] = getJoyValue<bool>(Keybinding::ext0);
    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::EXT1] = getJoyValue<bool>(Keybinding::ext1);
    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::EXT2] = getJoyValue<bool>(Keybinding::ext2);

    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT] = CONSTRAIN(getJoyValue<float>(Keybinding::joystick_left_front), -1.0f, 1.0f);
    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_SIDE] = CONSTRAIN(getJoyValue<float>(Keybinding::joystick_left_side), -1.0f, 1.0f);
    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT] = CONSTRAIN(getJoyValue<float>(Keybinding::joystick_right_front), -1.0f, 1.0f);
    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE] = CONSTRAIN(getJoyValue<float>(Keybinding::joystick_right_side), -1.0f, 1.0);

    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::L2] = MAP(float, CONSTRAIN(getJoyValue<float>(Keybinding::l2), -1.0f, 1.0f), _controller_config.trigger_range_min, _controller_config.trigger_range_max, 0.0f, 1.0f);
    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::R2] = MAP(float, CONSTRAIN(getJoyValue<float>(Keybinding::r2), -1.0f, 1.0f), _controller_config.trigger_range_min, _controller_config.trigger_range_max, 0.0f, 1.0f);

    float cross_temp = getJoyValue<float>(Keybinding::cross_front);
    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::CROSS_UP] = cross_temp > 0.0f ? true : false; //cross up
    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::CROSS_DOWN] = cross_temp < 0.0f ? true : false; //cross down
    cross_temp = getJoyValue<float>(Keybinding::cross_side);
    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::CROSS_LEFT] = cross_temp > 0.0f ? true : false; //cross left
    formatted_joy_msg.joy_data[rover_msgs::msg::Joy::CROSS_RIGHT] = cross_temp < 0.0f ? true : false; //cross right

    executeCustomSteps(&formatted_joy_msg);
    _pub_joy_formatted->publish(formatted_joy_msg);
    _last_formatted_joy_msg = formatted_joy_msg;
}

void JoyFormator::setControllerType(std::string controller_type_name)
{
    if (controller_type_name == std::string("DS4") || controller_type_name == std::string("PS4"))
    {
        RCLCPP_INFO(LOGGER, "Selected DS4 for keybinding mapping");

        _controller_config.buttons[Keybinding::a] = 0;
        _controller_config.buttons[Keybinding::b] = 1;
        _controller_config.buttons[Keybinding::x] = 3;
        _controller_config.buttons[Keybinding::y] = 2;
        _controller_config.buttons[Keybinding::l1] = 4;
        _controller_config.buttons[Keybinding::r1] = 5;
        _controller_config.buttons[Keybinding::joystick_left_push] = 11;
        _controller_config.buttons[Keybinding::joystick_right_push] = 12;
        _controller_config.buttons[Keybinding::ext0] = 8;
        _controller_config.buttons[Keybinding::ext1] = 9;

        _controller_config.axes[Keybinding::joystick_left_side] = 0;
        _controller_config.axes[Keybinding::joystick_left_front] = 1;
        _controller_config.axes[Keybinding::l2] = 2;
        _controller_config.axes[Keybinding::joystick_right_side] = 3;
        _controller_config.axes[Keybinding::joystick_right_front] = 4;
        _controller_config.axes[Keybinding::r2] = 5;
        _controller_config.axes[Keybinding::cross_front] = 7;
        _controller_config.axes[Keybinding::cross_side] = 6;

        _controller_config.trigger_range_min = 1.0f;
        _controller_config.trigger_range_max = -1.0f;

        _controller_config.joystick_dead_zone = 0.1f;
    }
    else if (controller_type_name == std::string("Logitech") || controller_type_name == std::string("Logitech Generic"))
    {
        RCLCPP_INFO_ONCE(LOGGER, "Selected Logitech Generic Controller for keybinding mapping");
        RCLCPP_WARN_ONCE(LOGGER, "**Logitech controllers are discouraged**\n"
                                 "When logitech controllers disconnects and reconnects, all their joystick values are "
                                 "set to 1.0 instead of 0.0. This node tries to handle this behavior but cannot "
                                 "garantee it will always detect it");

        _controller_config.buttons[Keybinding::a] = 1;
        _controller_config.buttons[Keybinding::b] = 2;
        _controller_config.buttons[Keybinding::x] = 0;
        _controller_config.buttons[Keybinding::y] = 3;
        _controller_config.buttons[Keybinding::l1] = 4;
        _controller_config.buttons[Keybinding::r1] = 5;
        _controller_config.buttons[Keybinding::l2] = 6;
        _controller_config.buttons[Keybinding::r2] = 7;
        _controller_config.buttons[Keybinding::ext0] = 8;
        _controller_config.buttons[Keybinding::ext1] = 9;
        _controller_config.buttons[Keybinding::joystick_left_push] = 10;
        _controller_config.buttons[Keybinding::joystick_right_push] = 11;

        _controller_config.axes[Keybinding::joystick_left_side] = 0;
        _controller_config.axes[Keybinding::joystick_left_front] = 1;

        _controller_config.axes[Keybinding::joystick_right_side] = 2;
        _controller_config.axes[Keybinding::joystick_right_front] = 3;

        _controller_config.axes[Keybinding::cross_front] = 5;
        _controller_config.axes[Keybinding::cross_side] = 4;

        _controller_config.trigger_range_min = 0.0f;
        _controller_config.trigger_range_max = 1.0f;

        _controller_config.joystick_dead_zone = 0.05f;

        _controller_config.custom_steps = &JoyFormator::customStepsLogitech;
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "controller_type parameter doesn't correspond to any keybinding. \"%s\" entered, possible"
                             " entry are: \"DS4\", \"PS4\"",
                     controller_type_name.c_str());
        throw ExeptBadLaunchParameters("Wrong controller_type");
    }
}

template <typename T>
T JoyFormator::getJoyValue(Keybinding::eKeybinding key_)
{
    if (_controller_config.buttons[key_] != -1)
    {
        return (T)_current_joy.buttons[_controller_config.buttons[key_]];
    }
    else if (_controller_config.axes[key_] != -1)
    {
        return (T)_current_joy.axes[_controller_config.axes[key_]];
    }
    else
    {
        return (T)0.0f;
    }
}

float JoyFormator::applyJoystickDeadZone(float value_)
{
    if (!IN_ERROR(value_, _controller_config.joystick_dead_zone, 0.0f))
    {
        return SIGN(value_) * MAP(float, abs(value_), _controller_config.joystick_dead_zone, 1.0f, 0.0f, 1.0f);
    }
    else
    {
        return 0.0f;
    }
}

bool JoyFormator::connected()
{
    return ((_last_joy_time + _timeout) > this->now());
}

void JoyFormator::executeCustomSteps(rover_msgs::msg::Joy *formatted_joy)
{
    if (_controller_config.custom_steps != NULL)
    {
        (this->*_controller_config.custom_steps)(formatted_joy);
    }
}

void JoyFormator::customStepsLogitech(rover_msgs::msg::Joy *formatted_joy)
{
    if (!_controller_detected)
    {
        return;
    }

    // When Logitech controllers reconnects after a disconnection, all it's
    // joystick values are 1.0 which is quite dangerous. This method detects
    // disconnection and overwrite commands with zeros until all joystick are
    // reseted.

    // Detect disconnect
    if (!connected())
    {
        _controller_reset_needed = true;
        *formatted_joy = rover_msgs::msg::Joy();
    }
    // Trying to catch disconnection->connection that the timeout couldn't catch
    // Should also work if controller was unpluged->pluged while the node was
    // offline
    else if ((_last_formatted_joy_msg.joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT] == 0.0f && formatted_joy->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT] == 1.0f) &&
             (_last_formatted_joy_msg.joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_SIDE] == 0.0f && formatted_joy->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_SIDE] == 1.0f) &&
             (_last_formatted_joy_msg.joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT] == 0.0f && formatted_joy->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT] == 1.0f) &&
             (_last_formatted_joy_msg.joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE] == 0.0f && formatted_joy->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE] == 1.0f))
    {
        _controller_reset_needed = true;
        *formatted_joy = rover_msgs::msg::Joy();
    }

    else if (_controller_reset_needed &&
             formatted_joy->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT] != 1.0f && //left front
             formatted_joy->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_SIDE] != 1.0f && //Lside
             formatted_joy->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT] != 1.0f && // RF
             formatted_joy->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE] != 1.0f)
    {
        _controller_reset_needed = false;

        RCLCPP_INFO(LOGGER, "Controller ready!");
    }

    if (_controller_reset_needed)
    {
        RCLCPP_WARN_THROTTLE(LOGGER, CLOCK, 5000, "\nLogitech controller disconnection detected!\n"
                                                  "**LET GO OF DEADMAN SWITCH**\n"
                                                  "Reset all joysticks by moving them a bit in all directions\n");
        *formatted_joy = rover_msgs::msg::Joy();
    }
}
