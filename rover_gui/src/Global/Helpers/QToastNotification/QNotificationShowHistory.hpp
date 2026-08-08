#ifndef QNOTIFICATION_SHOW_HISTORY
#define QNOTIFICATION_SHOW_HISTORY

#include "rclcpp/rclcpp.hpp"
#include "ui_NotificationHistoryPanel.h"

#include <QtWidgets/QWidget>
#include <QTimer>
#include <QStyle>
#include <QScrollBar>

namespace QHelper
{
    class QNotificationShowHistory : public QWidget
    {
        Q_OBJECT

        static constexpr size_t MARGIN = 5UL;

      public:
        QNotificationShowHistory();
        void showHistory(void);

      private:
        QRect _targetScreenRect;
        Ui::historyPanel _ui;
    };
}  // namespace QHelper

#endif  // QNOTIFICATION_SHOW_HISTORY
