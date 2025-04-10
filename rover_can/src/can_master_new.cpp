// ROS
#include "rclcpp/rclcpp.hpp"

#include "rover_can2/drivers/driver_linux.hpp"

class CanMasterNew : public rclcpp::Node
{
  public:
    CanMasterNew():
        Node("can_master_new")
    {
        
    };
};

int main(int argc, char* argv[])
{
    RoverCan2::Drivers::DriverLinux canDriver();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanMasterNew>());
    rclcpp::shutdown();
    return 0;
}