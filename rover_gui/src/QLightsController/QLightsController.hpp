#include "ui_LightsController.h"

#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/light.hpp>

class QLightsController : public QWidget
{
  private:
    static constexpr const char* TOPIC_LIGHTS_CTRL = "/rover/auxiliary/lights_control";
    static constexpr float FREQUENCY = 0.0F;

  public:
    explicit QLightsController(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);
    void toggleLightControl(bool checked_);
    void updateLightPWM();

  private:
    Ui::LightsController _ui;
    std::shared_ptr<rclcpp::Node> _node;
    rclcpp::Publisher<rover_msgs::msg::Light>::SharedPtr _pub_LightCmd;
};
