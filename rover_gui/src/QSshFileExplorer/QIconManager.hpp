#ifndef __ICON_MANAGER_HPP__
#define __ICON_MANAGER_HPP__

#include <QIcon>

/**
 * @brief Singleton to manage icons from certain extension
 * User should only call getInstance().getIcon(); 
 * 
 */
class QIconManager
{
  public:
    QIconManager(const QIconManager&) = delete;
    QIconManager& operator=(const QIconManager&) = delete;

    /**
     * @brief Return the singleton object
     * 
     * @return IconManager& 
     */
    static const QIconManager& getInstance(void);

    /**
     * @brief Return a QIcon& from an extension as a string
     * 
     * @param extension_ 
     * @return const QIcon& 
     */
    const QIcon& getIcon(const std::string& extension_) const;

  private:
    QIconManager();
    void initializeIcons(void);

    std::unordered_map<std::string, QIcon> _iconMap;
};

#endif  //__ICON_MANAGER_HPP__
