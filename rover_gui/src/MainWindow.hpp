#ifndef MAIN_WINDOWS_HPP
#define MAIN_WINDOWS_HPP

#include "QDashboard/QDashboard.hpp"
#include "QNavigation/QNavigation.hpp"
#include "QSideBar/QSideBar.hpp"
#include "QSshFileExplorer/QFileTransferWidget.hpp"
#include "QDeviceStatus/QDeviceStatus.hpp"

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
    QHBoxLayout _layout;
    QStackedWidget _stackedWidget;
    QShortcut _closeShortCut;

    QSideBar _sideBarWidget;
    QDashboard _dashboardWidget;
    QNavigation _navigationWidget;
    QFileTransferWidget _fileTransferWidget;
    QDeviceStatus _deviceStatusWidget;
};

#endif  // MAIN_WINDOWS_HPP
