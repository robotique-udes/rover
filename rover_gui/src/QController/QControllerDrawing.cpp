#include "QControllerDrawing.hpp"

QControllerDrawing::QControllerDrawing(QWidget* parent) : QWidget(parent)
{
    // Optionnel : s'assurer que le widget gère bien les fonds transparents ou personnalisés si besoin
    setAttribute(Qt::WA_StyledBackground, true);
}

void QControllerDrawing::setJoyData(const std::vector<float>& data)
{
    _joyData = data;
}

void QControllerDrawing::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    // Initialisation du painter pour dessiner sur ce widget
    QPainter painter(this);
    
    // Activer l'antialiasing pour des contours lisses
    painter.setRenderHint(QPainter::Antialiasing, true);

    // Sécurité : si le tableau est vide ou incomplet, on ne dessine rien
    if (_joyData.size() < rover_msgs::msg::Joy::MAX)
    {
        return;
    }

    float buttonA = _joyData[rover_msgs::msg::Joy::A];

    if (buttonA > 0.5f)
    {
        painter.setBrush(Qt::green); // Vert si le bouton A est pressé
        painter.setPen(QPen(Qt::darkGreen, 2));
    }
    else
    {
        painter.setBrush(Qt::gray);  // Gris si le bouton A est relâché
        painter.setPen(QPen(Qt::black, 2));
    }

    // Dessine un cercle représentant le bouton A à la position (50, 50) de taille 40x40
    painter.drawEllipse(50, 50, 40, 40);
    
    // Texte indicatif à côté du cercle
    painter.setPen(Qt::black);
    painter.drawText(100, 75, "Bouton A");


    float leftStick = _joyData[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT];

    // La largeur du rectangle varie en fonction de la valeur de l'axe du joystick (-1.0 à 1.0)
    int rectWidth = 60 + static_cast<int>(leftStick * 40);

    painter.setBrush(Qt::cyan);
    painter.setPen(QPen(Qt::blue, 2));

    // Dessine un rectangle positionné plus bas
    painter.drawRect(50, 120, rectWidth, 40);
    
    // Texte indicatif
    painter.setPen(Qt::black);
    painter.drawText(160, 145, QString("Stick Gauche: %1").arg(leftStick, 0, 'f', 2));
}