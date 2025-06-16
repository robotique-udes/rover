#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rover_msgs/msg/detail/camera_control__struct.hpp>
#include <rover_msgs/msg/camera_control.hpp>

class CameraTestPub : public rclcpp::Node
{
    /**/
  public:
    CameraTestPub():
        Node("camera_test_pub")
    {
        _publisher = this->create_publisher<rover_msgs::msg::CameraControl>("/rover/gps/position", 10);
        //_subscriber = this->create_subscription<rover_msgs::msg::CameraControl>(const std::string &topic_name, const rclcpp::QoS &qos, CallbackT &&callback)
        //timer_ = this->create_wall_timer(std::chrono::milli(500), std::bind(&GpsPublisher::publish_gps, this));
    }

  private:
    void CB_publishStatusPTZ()
    {
        //auto msg = rover_msgs::msg::Gps();

        //_publisher->publish(msg);
    }

    void CB_receivePTZcmd()
    {

    }

    rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _publisher;
    rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr _subscriber;

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraTestPub>());
    rclcpp::shutdown();
    return 0;
}
