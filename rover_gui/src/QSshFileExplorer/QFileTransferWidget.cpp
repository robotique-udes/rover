#include "QFileTransferWidget.hpp"

QFileTransferWidget::QFileTransferWidget(QWidget* parent_):
    _mainLayout(this),
    _splitter(parent_),
    _localFileSystem("phil", "localhost", "", &_splitter),
    _roverFileSystem("phil", "localhost", "/home/phil/Documents", &_splitter)
{
    _localFileSystem.getUI().cb_showHiddenFile->setChecked(false);
    _roverFileSystem.getUI().cb_showHiddenFile->setChecked(false);

    _splitter.addWidget(&_roverFileSystem);
    _splitter.addWidget(&_localFileSystem);

    _mainLayout.addWidget(&_splitter, 0, 0, 1, 1);
}

QFileTransferWidget::~QFileTransferWidget() {}
