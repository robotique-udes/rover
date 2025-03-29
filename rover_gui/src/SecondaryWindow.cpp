#include "SecondaryWindow.hpp"

#include <QStackedWidget>

SecondaryWindow::SecondaryWindow(std::shared_ptr<rclcpp::Node> guiNode_):
    QMainWindow(nullptr),
    _centralWidget(this),
    _layout(&_centralWidget),
    _tempLabel("Future camera window", this),
    _videoPlayerWidget(guiNode_,this)
{
    _tempLabel.setAlignment(Qt::AlignCenter);
    _layout.addWidget(&_tempLabel);
    _layout.addWidget(&_videoPlayerWidget);
    this->setCentralWidget(&_centralWidget);
}
