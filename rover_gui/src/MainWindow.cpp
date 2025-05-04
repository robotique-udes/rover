#include "MainWindow.hpp"

#include "Global/Constant/Keybinding.hpp"

#include <QStackedWidget>

MainWindow::MainWindow(std::shared_ptr<rclcpp::Node> guiNode_):
    QMainWindow(nullptr),
    _centralWidget(this),
    _layout(&_centralWidget),
    _stackedWidget(this),
    _closeShortCut(Constants::Keybinding::CLOSE_APP, this),
    _sideBarWidget(this),
    _dashboardWidget(guiNode_, this),
    _navigationWidget(guiNode_, this),
    _fileTransferWidget(this),
    _deviceStatusWidget(guiNode_, this)
{
    _stackedWidget.addWidget(&_dashboardWidget);
    _stackedWidget.addWidget(&_navigationWidget);
    _stackedWidget.addWidget(&_fileTransferWidget);
    _stackedWidget.addWidget(&_deviceStatusWidget);

    _layout.addWidget(&_sideBarWidget);
    _layout.addWidget(&_stackedWidget);
    this->setCentralWidget(&_centralWidget);

    connect(&_sideBarWidget, &QSideBar::switchPage, &_stackedWidget, &QStackedWidget::setCurrentIndex);
    connect(&_closeShortCut, &QShortcut::activated, this, &QWidget::close);
}

void MainWindow::closeEvent(QCloseEvent* event_)
{
    if (event_)
    {
        event_->accept();
    }
    QApplication::closeAllWindows();
}
