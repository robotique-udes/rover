#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/gps.hpp>

class GpsPublisher : public rclcpp::Node
{
  public:
    GpsPublisher():
        Node("gps_publisher"),
        latitude_(45.404476)
    {
        publisher_ = this->create_publisher<rover_msgs::msg::Gps>("/rover/gps/position", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&GpsPublisher::publish_gps, this));
    }

  private:
    void publish_gps()
    {
        auto msg = rover_msgs::msg::Gps();
        msg.latitude = 45.377775;
        msg.longitude = -71.92474;
        msg.heading = 120.0F;
        msg.satellite = 8;

        publisher_->publish(msg);
        // latitude_ += 0.0001;
    }

    rclcpp::Publisher<rover_msgs::msg::Gps>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double latitude_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsPublisher>());
    rclcpp::shutdown();
    return 0;
}
