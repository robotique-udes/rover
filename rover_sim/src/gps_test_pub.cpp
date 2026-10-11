#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/gps.hpp>

class GpsPublisher : public rclcpp::Node
{
  public:
    GpsPublisher():
        Node("gps_publisher"),
        latitude_(45.404476F),
        longitude_(-71.888351F),
        satellite_(8),
        fix_quality_(3),
        heading_deg_(0.0F)
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

        heading_deg_ += 20.0F;
        if (heading_deg_ >= 360.0F)
        {
            heading_deg_ -= 360.0F;
        }

        latitude_ += 0.000'1F;
        longitude_ += 0.000'1F;
    }

    rclcpp::Publisher<rover_msgs::msg::Gps>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    float latitude_;
    float longitude_;
    int satellite_;
    int fix_quality_;

    float heading_deg_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsPublisher>());
    rclcpp::shutdown();
    return 0;
}
