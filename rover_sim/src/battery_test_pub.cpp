#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/battery.hpp"

class BatteryPublisher : public rclcpp::Node
{
  public:
    BatteryPublisher():
        Node("battery_publisher")
    {
        _pourcentage = 100;
        _publisher = this->create_publisher<rover_msgs::msg::Battery>("/rover/auxiliary/battery", 10);
        _timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&BatteryPublisher::publish_battery, this));
    }

  private:
    void publish_battery()
    {
        auto msg = rover_msgs::msg::Battery();
        msg.pourcentage = _pourcentage;
        msg.valid = true;
        _publisher->publish(msg);
        _pourcentage -= 8;
        if (_pourcentage >= 100)
        {
            _pourcentage = 100;
        }
    }

    rclcpp::Publisher<rover_msgs::msg::Battery>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
    uint8_t _pourcentage;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatteryPublisher>());
    rclcpp::shutdown();
    return 0;
}