// ROS
#include "rclcpp/rclcpp.hpp"

#include "rover_can2/src/rover_can2/drivers/driver_linux.hpp"

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

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanMaster2>());
    rclcpp::shutdown();

    return 0;
}