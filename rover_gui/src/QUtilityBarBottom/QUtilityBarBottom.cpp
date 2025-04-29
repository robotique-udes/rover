#include "QUtilityBarBottom.hpp"

QUtilityBarBottom::QUtilityBarBottom(QWidget* parent_):
    QWidget(parent_)
{
    _ui.setupUi(this);

    this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    _ui.bottomFrame->setStyleSheet(R"(
        QFrame {
            background-color: #2e2e2e;
            border-radius: 5px;
            border: none;
        })");
                      
    connect(_ui.notificationHistory_PB,
            &QPushButton::clicked,
            this,
            [this]()
            {
                emit seeHistory();
            });
    
    this->raise();
}
