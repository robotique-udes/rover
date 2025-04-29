#ifndef QTOASTNOTIFICATION
#define QTOASTNOTIFICATION

// ROS
#include "rclcpp/rclcpp.hpp"
// QT

#include "UI_ToastNotification.h"
#include "UI_OpenNotifications.h"
#include "UI_NotificationHistory.h"
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <QPropertyAnimation>
#include <QParallelAnimationGroup>
#include <QTimer>
#include <QApplication>
#include <QGraphicsDropShadowEffect>
#include <QStyle>
#include <deque>

namespace QHelper
{

    class QToastNotification : public QWidget
    {
        Q_OBJECT

        static constexpr size_t NOTIF_DURATION_MS = 5'000UL;
        static constexpr size_t HISTORY_MAX_SIZE = 10U;
        static constexpr size_t MARGIN_NOTIF = 20U; 

      public:
        enum class eNotifType : size_t
        {
            ERROR = 0U,
            WARNING = 1U,
            INFO = 2U
        };

        struct sNotificationInfo
        {
#warning add time stamp
            // long long timeStamp;
            std::string title;
            std::string description;
            QToastNotification::eNotifType criticityLevel;
        };

        /**nameenelverspace
         * @brief Helper to send a notification (non blocking) from any thread.
         * The notification pops from the bottom right end of the main window.
         *
         * @param title_ Notification title
         * @param description_ Notification description
         * @param type_ Criticity level setting a corresponding icon to the notification
         * @param durationMs_ Duration of the notification (5s by default)
         */
        void notify(const std::string& title_,
                    const std::string& description_,
                    eNotifType type_,
                    size_t durationMs_ = NOTIF_DURATION_MS);

        static QToastNotification& getInstance();

        QRect targetScreenRect;
        std::deque<sNotificationInfo> history;

      private:
        QToastNotification();
        ~QToastNotification() = default;
        QToastNotification(const QToastNotification&) = delete;
        QToastNotification& operator=(const QToastNotification&) = delete;

        void setupUI();
        void setupAnimations();
        void setupScreenRect();

        void hideNotification();
        void saveNotifInfo(const sNotificationInfo& info_);

        Ui::ToastNotification _ui;
        QGraphicsDropShadowEffect _shadow;
        QPropertyAnimation _fadeInAnim;
        QPropertyAnimation _fadeOutAnim;
        QPropertyAnimation _slideInAnim;
        QPropertyAnimation _slideOutAnim;
        QPropertyAnimation _progressBarAnim;

        QTimer _closeTimer;
    };

    class QNotificationShowHistory : public QWidget
    {
        Q_OBJECT

        static constexpr size_t MARGIN = 5;

      public:
        QNotificationShowHistory();

        void showHistory();

      private:
        Ui::historyWidget _ui_mainWidget;

        QRect _targetScreenRect;
        QWidget _scrollAreaContainer;
        QVBoxLayout _scrollAreaLayout;
    };

    class QNotificationHistoryData : public QWidget
    {
        Q_OBJECT

      public:
        QNotificationHistoryData(std::string title_,
                                 std::string description_,
                                 QToastNotification::eNotifType type_);

      private:
        Ui::historySubWidget _ui_subWidget;
    };

}  // namespace QHelper

#endif