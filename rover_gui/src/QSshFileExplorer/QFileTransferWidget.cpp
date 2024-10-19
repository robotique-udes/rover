#include "QFileTransferWidget.hpp"

QFileTransferWidget::QFileTransferWidget(QWidget* parent_):
    _mainLayout(this),
    _localFileSystem("phil", "localhost", "", parent_),
    _roverFileSystem("phil", "localhost", "/home/phil/Documents", parent_)
{
    _localFileSystem.getUI().cb_showHiddenFile->setChecked(false);
    _roverFileSystem.getUI().cb_showHiddenFile->setChecked(false);

    _mainLayout.addWidget(&_roverFileSystem, 0, 0, 1, 1);
    _mainLayout.addWidget(&_localFileSystem, 0, 1, 1, 1);
}

QFileTransferWidget::~QFileTransferWidget() {}
