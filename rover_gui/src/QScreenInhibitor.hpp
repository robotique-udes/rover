#ifndef SCREEN_INHIBITOR_HPP
#define SCREEN_INHIBITOR_HPP

#include <QDBusInterface>

/**
 * @brief Construct a ScreenInhibitor object to prevent screen dimming during teleop, using DBus interface
 * @brief The Rover Base application becomes a priority over the screen saver
 */

class ScreenInhibitor
{
  public:
    ScreenInhibitor();
    ~ScreenInhibitor();

  private:
    uint _cookie = 0;
    bool _inhibited = false;
    QDBusInterface _interface = QDBusInterface("org.freedesktop.ScreenSaver",
                                               "/ScreenSaver",
                                               "org.freedesktop.ScreenSaver",
                                               QDBusConnection::sessionBus());
};

#endif  // SCREEN_INHIBITOR_HPP
