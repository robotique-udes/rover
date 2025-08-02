#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/gps.hpp>

class GpsPublisher : public rclcpp::Node
{
  public:
    GpsPublisher():
        Node("gps_publisher"),
        latitude_(45.404476),
        longitude_(-71.888351),
        satellite_(8),
        fix_quality_(3),
        heading_deg_(0.0)
    {
        publisher_ = this->create_publisher<rover_msgs::msg::Gps>("/rover/gps/position", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&GpsPublisher::publish_gps, this));
    }

  private:
    void publish_gps()
    {
        auto msg = rover_msgs::msg::Gps();
        msg.latitude = latitude_;
        msg.longitude = longitude_;
        msg.satellite = satellite_;
        msg.fix_quality = fix_quality_;

        msg.heading = heading_deg_;

        publisher_->publish(msg);

        latitude_ += 0.00001;

        heading_deg_ += 20.0F;
        if (heading_deg_ >= 360.0)
        {
            heading_deg_ -= 360.0;
        }

        latitude_ += 0.000'1;
        longitude_ += 0.000'1;
    }

    rclcpp::Publisher<rover_msgs::msg::Gps>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double latitude_;
    double longitude_;
    int satellite_;
    int fix_quality_;

    double heading_deg_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsPublisher>());
    rclcpp::shutdown();
    return 0;
}
