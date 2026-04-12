#include "MainWindow.hpp"

#include "Global/Constant/Keybinding.hpp"

MainWindow::MainWindow(std::shared_ptr<rclcpp::Node> guiNode_):
    QMainWindow(nullptr),
    _closeShortCut(Constants::Keybinding::CLOSE_APP, this),
    _topUtilityBar(guiNode_, this),
    _arbitrationWidget(guiNode_, this),
    _navigationWidget(guiNode_, this),
    _deviceStatusWidget(guiNode_, this),
    _ligthsController(guiNode_, this),
    _bmsDataWidget(guiNode_, this)
{
    this->setCentralWidget(&_centralWidget);
    _centralWidget.setLayout(&_verticalLayout);

    _verticalLayout.addWidget(&_topUtilityBar);
    _verticalLayout.setSpacing(0);
    _verticalLayout.setContentsMargins(0, 0, 0, 0);

    _topBarSeperator.setFrameShape(QFrame::HLine);
    _verticalLayout.addWidget(&_topBarSeperator);

    _verticalLayout.addLayout(&_layout);

    _bottomBarSeperator.setFrameShape(QFrame::HLine);
    _verticalLayout.addWidget(&_bottomBarSeperator);
    _verticalLayout.addWidget(&_bottomUtilityBar);

    _layout.addWidget(&_sideBarWidget);
    _layout.addWidget(&_mainTabWidget);
    _layout.addWidget(&_notificationHistoryWidget);

    _mainTabWidget.tabBar()->hide();

    connect(&_sideBarWidget, &QSideBar::switchPage, this, &MainWindow::onTabChange);
    connect(&_bottomUtilityBar,
            &QUtilityBarBottom::seeHistory,
            &_notificationHistoryWidget,
            &QHelper::QNotificationShowHistory::showHistory);
    connect(&_closeShortCut, &QShortcut::activated, this, &QWidget::close);

    this->onTabChange(QSideBar::eTabIndex::DASHBOARD);
}

void MainWindow::closeEvent(QCloseEvent* event_)
{
    if (event_)
    {
        event_->accept();
    }
    QApplication::closeAllWindows();
}

void MainWindow::onTabChange(QSideBar::eTabIndex index_)
{
    while (_mainTabWidget.count() != 0)
    {
        std::unique_ptr<QWidget> widgetContained = std::make_unique<QWidget>(_mainTabWidget.widget(0));
        _mainTabWidget.removeTab(0);
    }

    std::unique_ptr<QGridLayout> grid = std::make_unique<QGridLayout>();
    switch (index_)
    {
        default:
            RCLCPP_WARN(rclcpp::get_logger("GUI"),
                        "Unknown tab index %u for main window, defaulting to dashboard",
                        std::to_underlying(index_));
            [[fallthrough]];

        case QSideBar::eTabIndex::DASHBOARD:
            grid->addWidget(&_navigationWidget, 0, 0, 3, 1);
            grid->addWidget(&_arbitrationWidget, 0, 1);
            grid->addWidget(&_ligthsController, 1, 1);
            grid->addWidget(&_deviceStatusWidget, 2, 1);
            _bmsDataWidget.setGraphSize(200, 200);
            _bmsDataWidget.setCellContainerSize(90, 75);
            grid->addWidget(&_bmsDataWidget, 3, 1);

            grid->setColumnStretch(0, 6);
            grid->setColumnStretch(1, 1);
            _deviceStatusWidget.onDashboardPage();
            break;

        case QSideBar::eTabIndex::NAVIGATION:
            grid->addWidget(&_navigationWidget, 0, 1);
            break;

        case QSideBar::eTabIndex::DEVICE_STATUS:
            grid->addWidget(&_deviceStatusWidget, 0, 1);
            _deviceStatusWidget.onDeviceStatusPage();
            break;

        case QSideBar::eTabIndex::FILE_TRANSFER:
            grid->addWidget(&_fileTransferWidget, 0, 1);
            break;
        
        case QSideBar::eTabIndex::BMS_DATA:
            _bmsDataWidget.setGraphSize();
            _bmsDataWidget.setCellContainerSize();
            grid->addWidget(&_bmsDataWidget, 0 ,1);
            break;
    }

    std::unique_ptr<QWidget> widget = std::make_unique<QWidget>();
    widget->setLayout(grid.release());
    _mainTabWidget.addTab(widget.release(), "Main tab");
}
