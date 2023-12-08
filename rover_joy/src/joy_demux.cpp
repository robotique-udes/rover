#include "rclcpp/rclcpp.hpp"
#include "rovus_lib/rovus_exceptions.h"
#include "rovus_lib/macros.h"
#include "rover_msgs/msg/joy.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        // rclcpp::spin(std::make_shared<JoyFormator>());
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("Dead Node"), "Killing node on exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
