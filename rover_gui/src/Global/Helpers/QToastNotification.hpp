#ifndef QTOASTNOTIFICATION
#define QTOASTNOTIFICATION

// ROS
#include "rclcpp/rclcpp.hpp"
// QT
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <QPropertyAnimation>
#include <QTimer>
#include <QApplication>
#include "UI_ToastNotification.h"


class QToastNotification : public QWidget
{
        Q_OBJECT
    
    public:
        
        static QToastNotification& getInstance();
        
        void showMessage(const QString& message, int durationMs = 3000);

    private:
        QToastNotification();
        ~QToastNotification() = default;

        QToastNotification(const QToastNotification&) = delete;
        QToastNotification& operator=(const QToastNotification&) = delete;

        void setupAnimations();

        //Q_DISABLE_COPY(QSingletonToast)
        
        Ui::ToastNotification _ui;

        QPropertyAnimation _fadeInAnim;
        QPropertyAnimation _fadeOutAnim;
        QPropertyAnimation _slideAnim;
        QTimer _closeTimer; 

};

#endif