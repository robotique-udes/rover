#include "MainWindow.hpp"

#include <QStackedWidget>

MainWindow::MainWindow(std::shared_ptr<rclcpp::Node> guiNode_):
    QMainWindow(nullptr),
    _centralWidget(this),
    _layout(&_centralWidget),
    _stackedWidget(this),
    _sideBarWidget(this),
    _dashboardWidget(guiNode_, this),
    _navigationWidget(guiNode_, this)  //,
//_fileTransferWidget(this)
#warning put the file transfer widget back
{
    _stackedWidget.addWidget(&_dashboardWidget);
    _stackedWidget.addWidget(&_navigationWidget);
    //_stackedWidget.addWidget(&_fileTransferWidget);

    _layout.addWidget(&_sideBarWidget);
    _layout.addWidget(&_stackedWidget);
    this->setCentralWidget(&_centralWidget);

    connect(&_sideBarWidget, &QSideBar::switchPage, &_stackedWidget, &QStackedWidget::setCurrentIndex);
}
