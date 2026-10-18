#include "QController.hpp"

#include <QPixmap>

QController::QController(QWidget* parent_):
    QWidget(parent_)
{
    _ui.setupUi(this);

    connect(_ui.comboBox, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int index_)
    {
        QString iconPath;
        switch (index_)
        {
            case 1:
                iconPath = QStringLiteral(":/icons/Logitech_Controller.svg");
                break;
            case 2:
                iconPath = QStringLiteral(":/icons/XBox_Controller.svg");
                break;
            case 0:
            default:
                iconPath = QStringLiteral(":/icons/PS4_Controller.svg");
                break;
        }

        _ui.label->setPixmap(QPixmap(iconPath));
    });
}