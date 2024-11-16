#include "rcl_interfaces/msg/log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "rovus_lib/macros.h"
#include "rovus_lib/rovus_exceptions.h"

// =============================================================================
// This node listen to the topic set in the "debug_topic" parameter for log
// messages and prints them on a terminal. This node is used to listen to
// microcontrollers (running microROS) logging messages and print them on a the
// terminal to help with debugging
// =============================================================================

class JoyFormator : public rclcpp::Node
{
  private:
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr _sub_log;
    void callbackJoy(const rcl_interfaces::msg::Log& msg);

  public:
    JoyFormator();
    ~JoyFormator() {}
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<JoyFormator>());
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("Dead Node"), "Killing node on exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}

JoyFormator::JoyFormator(): Node("joy_formator")
{
    this->declare_parameter<std::string>("debug_topic", "not_set");
    _sub_log
        = this->create_subscription<rcl_interfaces::msg::Log>(this->get_parameter("debug_topic").as_string(),
                                                              1,
                                                              std::bind(&JoyFormator::callbackJoy, this, std::placeholders::_1));
}

void JoyFormator::callbackJoy(const rcl_interfaces::msg::Log& msg)
{
    std::string str = msg.msg.data();

    rcutils_log_location_t location;
    location.file_name = msg.file.data();
    location.function_name = msg.function.data();
    location.line_number = msg.line;

    rcutils_log(&location, msg.level, msg.name.c_str(), "%s", msg.msg.c_str());
}
