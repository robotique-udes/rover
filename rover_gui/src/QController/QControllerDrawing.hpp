#ifndef QCONTROLLERDRAWING_HPP
#define QCONTROLLERDRAWING_HPP

#include <QWidget>
#include <QPainter>
#include <vector>
#include <cmath>
#include <rover_msgs/msg/joy.hpp> // Pour accéder aux constantes de la manette (A, JOYSTICK_LEFT_FRONT, etc.)

class QControllerDrawing : public QWidget
{
    Q_OBJECT

  public:
    // Constructeur
    explicit QControllerDrawing(QWidget* parent = nullptr);

    // Méthode pour mettre à jour le tableau complet des 21 valeurs de la manette
    void setJoyData(const std::vector<float>& data);

  protected:
    // Méthode Qt appelée automatiquement lors d'un rafraîchissement (déclenché par update())
    void paintEvent(QPaintEvent* event) override;

  private:
    std::vector<float> _joyData; // Stocke le tableau de la manette
};

#endif 