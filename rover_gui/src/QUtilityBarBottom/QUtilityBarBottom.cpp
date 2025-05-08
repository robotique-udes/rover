#include "QUtilityBarBottom.hpp"

QUtilityBarBottom::QUtilityBarBottom(QWidget* parent_):
    QWidget(parent_)
{
    _ui.setupUi(this);

    connect(_ui.notificationHistory_PB,
            &QPushButton::clicked,
            this,
            [this]()
            {
                emit seeHistory();
            });

    this->raise();
}
