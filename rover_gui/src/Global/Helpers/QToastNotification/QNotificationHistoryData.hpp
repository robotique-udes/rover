#ifndef QNOTIFICATION_HISTORY_DATA
#define QNOTIFICATION_HISTORY_DATA

#include "rclcpp/rclcpp.hpp"
#include "QToastNotification.hpp"
#include "UI_NotificationHistory.h"

#include <mutex>
#include <QtWidgets/QWidget>
#include <QTimer>
#include <QStyle>
#include <QDateTime>

namespace QHelper
{
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

#endif //QNOTIFICATION_HISTORY_DATA
