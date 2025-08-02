#include "QLightsController.hpp"
#include <rover_lib2/helpers/constants.hpp>
#include <qpushbutton.h>
#include <qslider.h>
#include <qlabel.h>

QLightsController::QLightsController(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    _ui.setupUi(this);
    _pub_LightCmd = _node->create_publisher<rover_msgs::msg::Light>(TOPIC_LIGHTS_CTRL, QOS_DEFAULT);
    connect(_ui._slider_PWM, &QSlider::valueChanged, this, &QLightsController::updateLightPWM);
    connect(_ui._pb_lights, &QPushButton::clicked, this, &QLightsController::toggleLightControl);
}

void QLightsController::toggleLightControl(bool checked_)
{
    rover_msgs::msg::Light msg;

    if (checked_)
    {
        _ui._pb_lights->setText("Turn front lights off");
        msg.duty_cycle = std::clamp(static_cast<float>(_ui._slider_PWM->value()), 0.0f, 100.0f);
        msg.frequency = FREQUENCY;
    }
    else
    {
        _ui._pb_lights->setText("Turn front lights on");
        msg.duty_cycle = 0.0f;
        msg.frequency = 0.0f;
    }
    _pub_LightCmd->publish(msg);
}

void QLightsController::updateLightPWM(void)
{
    _ui._label_PWMprc->setText(QString::number(_ui._slider_PWM->value()) + "%");
    if (_ui._pb_lights->isChecked())
    {
        rover_msgs::msg::Light msg;
        msg.duty_cycle = std::clamp(static_cast<float>(_ui._slider_PWM->value()), 0.0f, 100.0f);
        msg.frequency = FREQUENCY;
        _pub_LightCmd->publish(msg);
    }
    else
    {
        rover_msgs::msg::Light msg;
        msg.duty_cycle = 0.0f;
        msg.frequency = 0.0f;
        _pub_LightCmd->publish(msg);
    }
}