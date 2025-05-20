#include "QFileTransferWidget.hpp"
#include "Global/Helpers/QHelpers.hpp"

QFileTransferWidget::QFileTransferWidget(QWidget* parent_):
    _mainLayout(this),
    _splitter(parent_),
    _localFileSystem(QHelper::getCurrentUserName(),
                     "localhost",
                     QStandardPaths::writableLocation(QStandardPaths::HomeLocation).toStdString(),
                     &_splitter),
    _roverFileSystem("rover", "192.168.144.20", "/home/rover", &_splitter)
{
    _localFileSystem.getUI().cb_showHiddenFile->setChecked(false);
    _roverFileSystem.getUI().cb_showHiddenFile->setChecked(false);

    _roverFileSystem.linkFriend(&_localFileSystem);
    _localFileSystem.linkFriend(&_roverFileSystem);

    _splitter.addWidget(&_localFileSystem);
    _splitter.addWidget(&_roverFileSystem);

    _mainLayout.addWidget(&_splitter, 0, 0, 1, 1);
}

QFileTransferWidget::~QFileTransferWidget() {}
