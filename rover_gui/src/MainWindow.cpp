#include "MainWindow.hpp"

#include "Global/Constant/Keybinding.hpp"

#include <QStackedWidget>

MainWindow::MainWindow(std::shared_ptr<rclcpp::Node> guiNode_):
    QMainWindow(nullptr),
    _centralWidget(this),
    _hBoxContainer(this),
    _layout(&_hBoxContainer),
    _verticalLayout(&_centralWidget),
    _stackedWidget(this),
    _closeShortCut(Constants::Keybinding::CLOSE_APP, this),
    _sideBarWidget(this),
    _bottomUtilityBar(this),
    _dashboardWidget(guiNode_, this),
    _navigationWidget(guiNode_, this),
    _fileTransferWidget(this)
{
    _stackedWidget.addWidget(&_dashboardWidget);
    _stackedWidget.addWidget(&_navigationWidget);

    _stackedWidget.addWidget(&_fileTransferWidget);

    _layout.addWidget(&_sideBarWidget);
    _layout.addWidget(&_stackedWidget);
    _layout.addWidget(&_notificationHistoryWidget);

    _hBoxContainer.setLayout(&_layout);

    _verticalLayout.addWidget(&_hBoxContainer);
    _verticalLayout.addWidget(&_bottomUtilityBar);

    _centralWidget.setLayout(&_verticalLayout);
    setCentralWidget(&_centralWidget);

    connect(&_sideBarWidget, &QSideBar::switchPage, &_stackedWidget, &QStackedWidget::setCurrentIndex);
    connect(&_bottomUtilityBar,
            &QUtilityBarBottom::seeHistory,
            &_notificationHistoryWidget,
            &QHelper::QNotificationShowHistory::showHistory);
    connect(&_closeShortCut, &QShortcut::activated, this, &QWidget::close);
}

void MainWindow::closeEvent(QCloseEvent* event_)
{
    if (event_)
    {
        event_->accept();
    }
    QHelper::QToastNotification::getInstance().cleanup();
    QApplication::closeAllWindows();
}
