#include "MainWindow.hpp"

#include "Global/Constant/Keybinding.hpp"

#include <QStackedWidget>
#include <memory>
#include <qgridlayout.h>
#include <qwidget.h>
#include <rclcpp/logger.hpp>

MainWindow::MainWindow(std::shared_ptr<rclcpp::Node> guiNode_):
    QMainWindow(nullptr),
    _centralWidget(this),
    _hBoxContainer(this),
    _layout(&_hBoxContainer),
    _verticalLayout(&_centralWidget),
    _mainTabWidget(this),
    _closeShortCut(Constants::Keybinding::CLOSE_APP, this),
    _sideBarWidget(this),
    _topUtilityBar(guiNode_, this),
    _bottomUtilityBar(this),
    _arbitrationWidget(guiNode_, this),
    _fileTransferWidget(this),
    _deviceStatusWidget(guiNode_, this),
    _navigationWidget(guiNode_, this)
{
    _mainTabWidget.tabBar()->hide();

    _arbitrationWidget.hide();
    _navigationWidget.hide();
    _deviceStatusWidget.hide();
    _fileTransferWidget.hide();

    _layout.addWidget(&_sideBarWidget);
    _layout.addWidget(&_mainTabWidget);
    _layout.addWidget(&_notificationHistoryWidget);

    _hBoxContainer.setLayout(&_layout);

    _verticalLayout.addWidget(&_topUtilityBar);
    _verticalLayout.addWidget(&_hBoxContainer);
    _verticalLayout.addWidget(&_bottomUtilityBar);

    _centralWidget.setLayout(&_verticalLayout);
    this->setCentralWidget(&_centralWidget);

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
            grid->addWidget(&_navigationWidget, 0, 0);
            grid->addWidget(&_arbitrationWidget, 0, 1);
            grid->addWidget(&_deviceStatusWidget, 2, 0);
            _navigationWidget.show();
            _arbitrationWidget.show();
            _deviceStatusWidget.hideControls();
            _deviceStatusWidget.show();
            break;

        case QSideBar::eTabIndex::NAVIGATION:
            grid->addWidget(&_navigationWidget, 0, 1);
            _navigationWidget.show();
            break;

        case QSideBar::eTabIndex::DEVICE_STATUS:
            grid->addWidget(&_deviceStatusWidget, 0, 1);
            _deviceStatusWidget.showControls();
            _deviceStatusWidget.show();
            break;

        case QSideBar::eTabIndex::FILE_TRANSFER:
            grid->addWidget(&_fileTransferWidget, 0, 1);
            _fileTransferWidget.show();
            break;
    }

    std::unique_ptr<QWidget> widget = std::make_unique<QWidget>();
    widget->setLayout(grid.release());
    _mainTabWidget.addTab(widget.release(), "Main tab");
}
