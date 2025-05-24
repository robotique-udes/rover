#ifndef SCREEN_INHIBITOR_HPP
#define SCREEN_INHIBITOR_HPP

#include <QDBusInterface>
#include <QDBusReply>

/**
 * @brief Construct a ScreenInhibitor object to prevent screen dimming during teleop, using DBus interface
 * @brief The Rover Base application becomes a priority over the screen saver
 */

class ScreenInhibitor
{
  public:
    ScreenInhibitor():
        interface("org.freedesktop.ScreenSaver", "/ScreenSaver", "org.freedesktop.ScreenSaver", QDBusConnection::sessionBus())
    {
        if (!interface.isValid())
        {
            RCLCPP_WARN(rclcpp::get_logger("GUI"), "Failed to connect to ScreenSaver interface");
            return;
        }

        QDBusReply<uint> reply = interface.call("Inhibit", "Rover GUI", "Screen dimming during rover teleop can cause issues");
        if (reply.isValid())
        {
            cookie = reply.value();
            RCLCPP_INFO(rclcpp::get_logger("GUI"), "Inhibition cookie activated: %u", cookie);
            inhibited = true;
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("GUI"), "Inhibition failed: ");
        }
    }

    ~ScreenInhibitor()
    {
        if (inhibited)
        {
            if (interface.isValid())
            {
                interface.call("UnInhibit", cookie);
                RCLCPP_INFO(rclcpp::get_logger("GUI"), "Inhibition cookie released: %u", cookie);
            }
        }
    }

  private:
    uint cookie = 0;
    bool inhibited = false;
    QDBusInterface interface;
};

#endif  // SCREEN_INHIBITOR_HPP