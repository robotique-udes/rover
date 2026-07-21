#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/bms_data.hpp"
#include <cmath>
#include <memory>
#include <vector>

class BmsPublisher : public rclcpp::Node
{
  public:
    BmsPublisher():
        Node("bms_mock_publisher"),
        _tick(0)
    {
        _publisher = this->create_publisher<rover_msgs::msg::BmsData>("rover/auxiliary/bms_data", 10);
        _timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&BmsPublisher::publish_data, this));
        RCLCPP_INFO(this->get_logger(), "BMS mock publisher started on 'rover/auxiliary/bms_data'");
    }

  private:
    void publish_data()
    {
        auto msg = rover_msgs::msg::BmsData();

        msg.battery_amps = 5.0f + 10.0f * static_cast<float>(std::sin(_tick * 0.05)) * -100;

        const std::size_t NUM_CELLS = 6;
        msg.cell_volt.resize(NUM_CELLS);    
        for (std::size_t i = 0; i < NUM_CELLS; ++i)
        {
            msg.cell_volt[i] = static_cast<uint16_t>(3700 + 500 * std::sin(_tick * 0.03 + static_cast<double>(i) * 0.5));
        }

        _publisher->publish(msg);

        RCLCPP_DEBUG(this->get_logger(),
                     "Published: amps=%.2f  cells[0]=%u mV  cells[%zu]=%u mV",
                     msg.battery_amps,
                     msg.cell_volt.front(),
                     NUM_CELLS - 1,
                     msg.cell_volt.back());

        ++_tick;
    }

    rclcpp::Publisher<rover_msgs::msg::BmsData>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
    uint64_t _tick;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BmsPublisher>());
    rclcpp::shutdown();
    return 0;
}