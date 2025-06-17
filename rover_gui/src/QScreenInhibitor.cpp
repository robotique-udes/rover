#include "QScreenInhibitor.hpp"

#include <QDBusReply>
#include <rclcpp/rclcpp.hpp>

ScreenInhibitor::ScreenInhibitor()
{
    if (!_interface.isValid())
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Failed to connect to ScreenSaver interface");
        return;
    }

    QDBusReply<uint> reply = _interface.call("Inhibit", "Rover GUI", "Screen dimming during rover teleop can cause issues");
    if (reply.isValid())
    {
        _cookie = reply.value();
        RCLCPP_INFO(rclcpp::get_logger("GUI"), "Inhibition cookie activated: %u", _cookie);
        _inhibited = true;
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Inhibition failed: ");
    }
}

ScreenInhibitor::~ScreenInhibitor()
{
    if (_inhibited)
    {
        if (_interface.isValid())
        {
            _interface.call("UnInhibit", _cookie);
            RCLCPP_INFO(rclcpp::get_logger("GUI"), "Inhibition cookie released: %u", _cookie);
        }
    }
}
