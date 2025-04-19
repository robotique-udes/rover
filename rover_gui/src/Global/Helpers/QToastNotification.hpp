#ifndef QTOASTNOTIFICATION
#define QTOASTNOTIFICATION

// ROS
#include "rclcpp/rclcpp.hpp"
// QT
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <QPropertyAnimation>
#include <QTimer>
#include "UI_ToastNotification.h"


class QToastNotification : public QWidget
{
        Q_OBJECT
    
    public:
        QToastNotification(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);
        void showMessage(const QString& message, int durationMs);

    private:

        std::shared_ptr<rclcpp::Node> _node;
        Ui::ToastNotification _ui;

        QPropertyAnimation _fadeInAnim;
        QPropertyAnimation _fadeOutAnim;
        QPropertyAnimation _slideAnim;
        QTimer _closeTimer; 

        void setupAnimations();

};


#endif // QTOASTNOTIFICATION