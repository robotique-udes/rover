#include "QController.hpp"

#include <QPixmap>
#include "rover_lib2/helpers/constants.hpp"

QController::QController(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(std::move(guiNode_))
{
    _ui.setupUi(this);

    _joySub = _node->create_subscription<rover_msgs::msg::Joy>(
        "/base/joy/secondary_formatted",
        QOS_DEFAULT,
        [this](const rover_msgs::msg::Joy& msg)
        {
            const float buttonA = msg.joy_data[rover_msgs::msg::Joy::A];
            const float leftStick = msg.joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT];

            emit joyInputReceived(buttonA, leftStick);
            
            RCLCPP_INFO(
                _node->get_logger(),
                "A: %.0f, left stick: %.2f",
                buttonA,
                leftStick);
                
        });


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

    connect(this, &QController::joyInputReceived, this,
    [this](float buttonA, float leftStick)
    {
        _ui.label_2->setText(
            QString("Button A: %1").arg(buttonA, 0, 'f', 0));

    },
    Qt::QueuedConnection);


}