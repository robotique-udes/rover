#ifndef MAIN_WINDOWS_HPP
#define MAIN_WINDOWS_HPP

#include "QDashboard/QDashboard.hpp"
#include "QNavigation/QNavigation.hpp"
#include "QSideBar/QSideBar.hpp"
#include "QSshFileExplorer/QFileTransferWidget.hpp"
#include "Global/Helpers/QToastNotification.hpp"

#include <QStackedWidget>
#include <QMainWindow>

#include "rclcpp/rclcpp.hpp"

class MainWindow : public QMainWindow
{
    Q_OBJECT

  public:
    explicit MainWindow(std::shared_ptr<rclcpp::Node> guiNode_);

  private:
    QWidget _centralWidget;
    QHBoxLayout _layout;
    QStackedWidget _stackedWidget;

    QSideBar _sideBarWidget;
    QDashboard _dashboardWidget;
    QNavigation _navigationWidget;
    QHelper::QNotificationShowHistory _notificationHistoryWidget;

    // QFileTransferWidget _fileTransferWidget;
};

#endif  // MAIN_WINDOWS_HPP
