#ifndef __ICON_MANAGER_HPP__
#define __ICON_MANAGER_HPP__

#include <QIcon>

class IconManager
{
  public:
    static std::map<std::string, QIcon>& getIconMap();

  private:
    static void initializeIcons();

    static bool _isInit;
    static std::map<std::string, QIcon> _iconMap;
};

#endif //__ICON_MANAGER_HPP__
