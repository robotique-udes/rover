#include "MainWindow.hpp"

#include <QStackedWidget>

MainWindow::MainWindow(std::shared_ptr<rclcpp::Node> guiNode_):
    QMainWindow(nullptr),
    _centralWidget(this),
    _hBoxContainer(this),
    _layout(this),
    _verticalLayout(this),
    _stackedWidget(this),
    _sideBarWidget(this),
    _bottomUtilityBar(this),
    _dashboardWidget(guiNode_, this),
    _navigationWidget(guiNode_, this)

//_fileTransferWidget(this)
#warning put the file transfer widget back
{
    _stackedWidget.addWidget(&_dashboardWidget);
    _stackedWidget.addWidget(&_navigationWidget);

    //_stackedWidget.addWidget(&_fileTransferWidget);

    _layout.addWidget(&_sideBarWidget);
    _layout.addWidget(&_stackedWidget);

    _hBoxContainer.setLayout(&_layout);

    _verticalLayout.addWidget(&_hBoxContainer);
    _verticalLayout.addWidget(&_bottomUtilityBar);

    _bottomUtilityBar.setFixedHeight(30);

    setCentralWidget(&_centralWidget);
    _centralWidget.setLayout(&_verticalLayout);


    connect(&_sideBarWidget, &QSideBar::switchPage, &_stackedWidget, &QStackedWidget::setCurrentIndex);
    connect(&_bottomUtilityBar, &QUtilityBarBottom::seeHistory, &_notificationHistoryWidget, &QHelper::QNotificationShowHistory::showHistory);
}
