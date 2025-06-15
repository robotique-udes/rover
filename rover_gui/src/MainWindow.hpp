#ifndef MAIN_WINDOWS_HPP
#define MAIN_WINDOWS_HPP

#include "QArbitration/QArbitration.hpp"
#include "QNavigation/QNavigation.hpp"
#include "Global/Helpers/QToastNotification/QNotificationShowHistory.hpp"
#include "QSideBar/QSideBar.hpp"
#include "QSshFileExplorer/QFileTransferWidget.hpp"
#include "QUtilityBarBottom/QUtilityBarBottom.hpp"
#include "QUtilityBarTop/QUtilityBarTop.hpp"
#include "QDeviceStatus/QDeviceStatus.hpp"

#include <QStackedWidget>
#include <QMainWindow>
#include <QShortcut>
#include <qgridlayout.h>

class MainWindow : public QMainWindow
{
    Q_OBJECT

  public:
    explicit MainWindow(std::shared_ptr<rclcpp::Node> guiNode_);

  private slots:
    void onTabChange(QSideBar::eTabIndex index_);

  private:
    void closeEvent(QCloseEvent* event_) override;

    QWidget _centralWidget;
    QWidget _hBoxContainer;
    QHBoxLayout _layout;
    QVBoxLayout _verticalLayout;
    QTabWidget _mainTabWidget;
    QShortcut _closeShortCut;

    QSideBar _sideBarWidget;
    QUtilityBarTop _topUtilityBar;
    QUtilityBarBottom _bottomUtilityBar;
    QHelper::QNotificationShowHistory _notificationHistoryWidget;

    QArbitration _arbitrationWidget;
    QFileTransferWidget _fileTransferWidget;
    QDeviceStatus _deviceStatusWidget;
    QNavigation _navigationWidget;
};

#endif  // MAIN_WINDOWS_HPP
