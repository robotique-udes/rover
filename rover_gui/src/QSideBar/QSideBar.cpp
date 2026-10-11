#include "QSideBar.hpp"

QSideBar::QSideBar(QWidget* parent_):
    QWidget(parent_)
{
    _ui.setupUi(this);

    // clang-format off
    connect(_ui.pb_dashboard, &QPushButton::clicked, this, [this]() { emit this->switchPage(eTabIndex::DASHBOARD); });
    connect(_ui.pb_navigation, &QPushButton::clicked, this, [this]() { emit this->switchPage(eTabIndex::NAVIGATION); });
    connect(_ui.pb_deviceStatus, &QPushButton::clicked, this, [this]() { emit this->switchPage(eTabIndex::DEVICE_STATUS); });
    connect(_ui.pb_fileTransfer, &QPushButton::clicked, this, [this]() { emit this->switchPage(eTabIndex::FILE_TRANSFER); });
    connect(_ui.pb_bmsData, &QPushButton::clicked, this, [this]() { emit this->switchPage(eTabIndex::BMS_DATA); });
    connect(_ui.pb_science, &QPushButton::clicked, this, [this]() { emit this->switchPage(eTabIndex::SCIENCE); });
    // clang-format on
}
