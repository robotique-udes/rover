#ifndef SCREENINHIBITOR_HPP
#define SCREENINHIBITOR_HPP

#include <QDBusInterface>
#include <QDBusReply>
#include <QDebug>

class ScreenInhibitor
{
  public:
    ScreenInhibitor()
    {
        QDBusInterface interface("org.freedesktop.ScreenSaver",
                                 "/ScreenSaver",
                                 "org.freedesktop.ScreenSaver",
                                 QDBusConnection::sessionBus());

        if (!interface.isValid())
        {
            RCLCPP_WARN(rclcpp::get_logger("GUI"), "Failed to connect to ScreenSaver interface");
            return;
        }

        QDBusReply<uint> reply = interface.call("Inhibit", "Rover GUI", "Running critical task");
        if (reply.isValid())
        {
            cookie = reply.value();
            qDebug() << "Screen inhibition activated, cookie:" << cookie;
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
            QDBusInterface interface("org.freedesktop.ScreenSaver",
                                     "/ScreenSaver",
                                     "org.freedesktop.ScreenSaver",
                                     QDBusConnection::sessionBus());
            if (interface.isValid())
            {
                interface.call("UnInhibit", cookie);
                qDebug() << "Screen inhibition released";
            }
        }
    }

  private:
    uint cookie = 0;
    bool inhibited = false;
};

#endif  // SCREENINHIBITOR_HPP
