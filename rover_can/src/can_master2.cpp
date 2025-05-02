// ROS
#include <rclcpp/rclcpp.hpp>

#include "rover_can2/drivers/driver_linux.hpp"
#include "rover_lib2/helpers/log.hpp"

DEFINE_LOG_NODE(CanMaster, Logger::eNodeState::ON);

class CanMaster2 : public rclcpp::Node
{
  public:
    CanMaster2():
        Node("can_master2")
    {
        
    };
};

int main(int argc, char* argv[])
{
    RoverCan2::Drivers::DriverLinux canDriver;
    canDriver.__init();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanMaster2>());
    rclcpp::shutdown();

    return 0;
}