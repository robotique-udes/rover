#include "QUtilityBarBottom.hpp"

QUtilityBarBottom::QUtilityBarBottom(QWidget* parent_):
    QWidget(parent_)
{
    _ui.setupUi(this);
    this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    _ui.bottomFrame->setStyleSheet("QFrame {"
                         "background-color: #2e2e2e;  /* Dark grey color */"
                         "border-radius: 5px;"        /* Optional: for rounded corners */
                         "border: none;"              /* Optional: remove any border */
                         "}");    
                                    qDebug("connecting");
     
    connect(_ui.notificationHistory_PB, &QPushButton::clicked, this, [this]() { emit seeHistory(); });
}
