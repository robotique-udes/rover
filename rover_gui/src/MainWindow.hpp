#ifndef MAIN_WINDOWS_HPP
#define MAIN_WINDOWS_HPP

#include "QDashboard/QDashboard.hpp"
#include "QNavigation/QNavigation.hpp"
#include "Global/Helpers/QToastNotification/QNotificationShowHistory.hpp"
#include "QSideBar/QSideBar.hpp"
#include "QSshFileExplorer/QFileTransferWidget.hpp"
#include "QUtilityBarBottom/QUtilityBarBottom.hpp"
#include "QTopUtilityBar/QTopUtilityBar.hpp"

#include <QStackedWidget>
#include <QMainWindow>
#include <QShortcut>

#include "rclcpp/rclcpp.hpp"

class MainWindow : public QMainWindow
{
    Q_OBJECT

  public:
    explicit MainWindow(std::shared_ptr<rclcpp::Node> guiNode_);

  private:
    void closeEvent(QCloseEvent* event_) override;

    QWidget _centralWidget;
    QWidget _hBoxContainer;
    QHBoxLayout _layout;
    QVBoxLayout _verticalLayout;
    QStackedWidget _stackedWidget;
    QShortcut _closeShortCut;

    QSideBar _sideBarWidget;
    QTopUtilityBar _topUtilityBar;
    QUtilityBarBottom _bottomUtilityBar;
    QDashboard _dashboardWidget;
    QNavigation _navigationWidget;
    QHelper::QNotificationShowHistory _notificationHistoryWidget;

    QFileTransferWidget _fileTransferWidget;
};

#endif  // MAIN_WINDOWS_HPP
