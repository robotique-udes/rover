#include "QSideBar.hpp"

QSideBar::QSideBar(QWidget* parent_):
    QWidget(parent_)
{
    _ui.setupUi(this);

    // clang-format off
    connect(_ui.pb_dashboard, &QPushButton::clicked, this, [this]() { emit this->switchPage(0); });
    connect(_ui.pb_navigation, &QPushButton::clicked, this, [this]() { emit this->switchPage(1); });
    connect(_ui.pb_fileTransfer, &QPushButton::clicked, this, [this]() { emit this->switchPage(2); });
    // clang-format on
}
