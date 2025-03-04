#include "SecondaryWindow.hpp"

SecondaryWindow::SecondaryWindow():
    QMainWindow(nullptr),
    _centralWidget(this),
    _layout(&_centralWidget),
    _tempLabel("Future camera window", this)
{
    _tempLabel.setAlignment(Qt::AlignCenter);
    _layout.addWidget(&_tempLabel);
    this->setCentralWidget(&_centralWidget);
}
