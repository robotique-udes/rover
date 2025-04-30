#include "SecondaryWindow.hpp"

#include "Global/Constant/Keybinding.hpp"

#include <QStackedWidget>
#include <QCloseEvent>

SecondaryWindow::SecondaryWindow(std::shared_ptr<rclcpp::Node> guiNode_):
    QMainWindow(nullptr),
    _centralWidget(this),
    _layout(&_centralWidget),
    _closeShortCut(Constants::Keybinding::CLOSE_APP, this),
    _videoPlayerWidget(guiNode_, this)
{
    _layout.addWidget(&_videoPlayerWidget);
    this->setCentralWidget(&_centralWidget);

    connect(this, &QWidget::destroyed, qApp, &QCoreApplication::quit);
    connect(&_closeShortCut, &QShortcut::activated, this, &QWidget::close);
}

void SecondaryWindow::closeEvent(QCloseEvent* event_)
{
    if (event_)
    {
        event_->accept();
    }
    QApplication::closeAllWindows();
}
