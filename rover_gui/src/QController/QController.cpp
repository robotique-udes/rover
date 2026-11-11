#include "QController.hpp"
#include "QControllerDrawing.hpp"
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
            
            // On extrait le tableau brut (fixed-size array) du message pour en faire un std::vector<float>
            std::vector<float> joyData(msg.joy_data.begin(), msg.joy_data.end());

            // On émet le signal Qt. C'est le pont sécurisé vers le thread principal de l'interface graphique.
            emit joyInputReceived(joyData);
            
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

    // 4. RÉCEPTION ET TRAITEMENT DES DONNÉES DE LA MANETTE DANS L'UI
    // Grâce à Qt::QueuedConnection, ce code s'exécute proprement dans le thread principal de Qt
    connect(this, &QController::joyInputReceived, this,
    [this](const std::vector<float>& joyData)
    {
        // Vérification de la taille du tableau
        if (joyData.size() < rover_msgs::msg::Joy::MAX)
        {
            return;
        }

        // On transmet tout le tableau de 21 éléments au widget de dessin personnalisé
        // (Assurez-vous d'avoir un widget nommé 'drawingWidget' promu dans votre UI)
        _ui.drawingWidget->setJoyData(joyData);

        // On demande à Qt de redessiner le widget graphique instantanément
        _ui.drawingWidget->update();

    },
    Qt::QueuedConnection);
}