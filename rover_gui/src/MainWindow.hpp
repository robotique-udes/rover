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
#include "QLightsController/QLightsController.hpp"
#include "QBmsData/QBmsData.hpp"

#include <QMainWindow>
#include <QShortcut>
#include <QFrame>

class MainWindow : public QMainWindow
{
    Q_OBJECT

  public:
    explicit MainWindow(std::shared_ptr<rclcpp::Node> guiNode_);

  private slots:
    void onTabChange(QSideBar::eTabIndex index_);

  private:
    void closeEvent(QCloseEvent* event_) override;

    QShortcut _closeShortCut;

    QWidget _centralWidget = QWidget(this);
    QHBoxLayout _layout;
    QVBoxLayout _verticalLayout = QVBoxLayout(&_centralWidget);
    QTabWidget _mainTabWidget = QTabWidget(this);

    QUtilityBarTop _topUtilityBar;
    QUtilityBarBottom _bottomUtilityBar = QUtilityBarBottom(this);

    QSideBar _sideBarWidget = QSideBar(this);
    QHelper::QNotificationShowHistory _notificationHistoryWidget;

    QArbitration _arbitrationWidget;
    QNavigation _navigationWidget;
    QDeviceStatus _deviceStatusWidget;
    QFileTransferWidget _fileTransferWidget = QFileTransferWidget(this);
    QLightsController _ligthsController;
    QBmsData _bmsDataWidget;

    QFrame _topBarSeperator = QFrame(this);
    QFrame _bottomBarSeperator = QFrame(this);
};

#endif  // MAIN_WINDOWS_HPP
