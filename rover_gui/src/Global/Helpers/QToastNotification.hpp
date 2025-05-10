#ifndef QTOASTNOTIFICATION
#define QTOASTNOTIFICATION

#include "rclcpp/rclcpp.hpp"
#include "UI_ToastNotification.h"
#include "UI_NotificationHistoryPanel.h"
#include "UI_NotificationHistory.h"

#include <mutex>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <QPropertyAnimation>
#include <QParallelAnimationGroup>
#include <QTimer>
#include <QApplication>
#include <QGraphicsDropShadowEffect>
#include <QStyle>
#include <QDateTime>
#include <QScrollBar>
#include <deque>
#include <qscrollarea.h>

namespace QHelper
{

    class QToastNotification : public QWidget
    {
        Q_OBJECT

        static constexpr size_t NOTIF_DURATION_MS = 5'000UL;
        static constexpr size_t HISTORY_MAX_SIZE = 50U;
        static constexpr size_t MARGIN_NOTIF = 20U;

      public:
        enum class eNotifType : size_t
        {
            ERROR = 0U,
            WARNING = 1U,
            INFO = 2U,
            SUCCESS = 3U
        };

        struct sNotificationInfo
        {
            QTime timeStamp;
            QString title;
            QString description;
            QToastNotification::eNotifType severityLevel;
        };

        /**
         * @brief Helper to send a notification (non blocking).
         * The notification pops from the bottom right end of the main window.
         *
         * @param title_ Notification title
         * @param description_ Notification description
         * @param type_ severity level setting a corresponding icon to the notification
         * @param durationMs_ Duration of the notification (5s by default)
         */
        void notifyFromAnyThread(const QString& title_,
                                 const QString& description_,
                                 eNotifType type_,
                                 size_t durationMs_ = NOTIF_DURATION_MS);
        static QToastNotification& getInstance();

        QRect getTargetScreenRect(void);
        std::deque<sNotificationInfo>& getHistory(void);
        void setTargetScreenRect(QRect targetScreenRect_);
        void setHistory(std::deque<sNotificationInfo> history_);

      private:
        void notify(const QString& title_, const QString& description_, eNotifType type_, size_t durationMs_ = NOTIF_DURATION_MS);
        QToastNotification();
        QToastNotification(const QToastNotification&) = delete;
        QToastNotification& operator=(const QToastNotification&) = delete;

        void setupUI(void);
        void setupAnimations(void);
        void setupScreenRect(void);

        void hideNotification(void);
        void saveNotifInfo(const sNotificationInfo& info_);

        QRect _targetScreenRect;
        std::deque<sNotificationInfo> _history;
        std::mutex _historyMutex;
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
        void showHistory(void);

      private:
        QRect _targetScreenRect;
        Ui::historyPanel _ui;
    };

    class QNotificationHistoryData : public QWidget
    {
        Q_OBJECT

      public:
        QNotificationHistoryData(QTime timeStamp_, QString title_, QString description_, QToastNotification::eNotifType type_);

      private:
        void setStyle(void);
        Ui::historySubWidget _ui;
    };

}  // namespace QHelper

#endif
