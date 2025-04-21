#ifndef QTOASTNOTIFICATION
#define QTOASTNOTIFICATION

// ROS
#include "rclcpp/rclcpp.hpp"
// QT
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <QPropertyAnimation>
#include <QParallelAnimationGroup>
#include <QTimer>
#include <QApplication>
#include "UI_ToastNotification.h"
#include <QGraphicsDropShadowEffect>
#include <QStyle>

class QToastNotification : public QWidget
{
    Q_OBJECT

    static constexpr uint64_t NOTIF_DURATION_MS = 5'000UL;

  public:
    enum class eNotifType : uint8_t
    {
        ERROR = 0u,
        WARNING = 1u,
        INFO = 2u
    };

    static QToastNotification& getInstance();

    void notify(const QString& title, const QString& description_, eNotifType type_, int durationMs_ = NOTIF_DURATION_MS);

  private:
    QToastNotification();
    ~QToastNotification() = default;

    void setupUI();
    void setupAnimations();
    void setupScreenRect();
    void hideNotification();

    QToastNotification(const QToastNotification&) = delete;
    QToastNotification& operator=(const QToastNotification&) = delete;



    // Q_DISABLE_COPY(QSingletonToast)

    Ui::ToastNotification _ui;
    QRect _targetScreenRect;
    QGraphicsDropShadowEffect _shadow;
    QPropertyAnimation _fadeInAnim;
    QPropertyAnimation _fadeOutAnim;
    QPropertyAnimation _slideInAnim;
    QPropertyAnimation _slideOutAnim;
    QPropertyAnimation _progressBarAnim;

    QParallelAnimationGroup _animationGroup;

    QTimer _closeTimer;
};

#endif