#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/wifi_connection.hpp"

class ConnectionSpeedPublisher : public rclcpp::Node
{
  public:
    ConnectionSpeedPublisher():
        Node("connection_speed_publisher")
    {
        _speed_connection = 1;
        _connection_quality = -50;
        _publisher = this->create_publisher<rover_msgs::msg::WifiConnection>("/rover/auxiliary/connection_speed", 10);
        _timer = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&ConnectionSpeedPublisher::publish_connection_speed, this));
    }

  private:
    void publish_connection_speed()
    {
        auto msg = rover_msgs::msg::WifiConnection();
        msg.speed_connection = _speed_connection;
        msg.rssi = _connection_quality;
        msg.valid = true;
        _publisher->publish(msg);
        _speed_connection += 1;
        _connection_quality -= 2;
        if (_connection_quality <= -90)
        {
            _connection_quality = -50;
        }
        if (_speed_connection >= 10.9)
        {
            _speed_connection = 1;
        }
    }

    rclcpp::Publisher<rover_msgs::msg::WifiConnection>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
    float _speed_connection;
    float _connection_quality;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConnectionSpeedPublisher>());
    rclcpp::shutdown();
    return 0;
}