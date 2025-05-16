#include "QNotificationShowHistory.hpp"
#include "QToastNotification.hpp"
#include "QNotificationHistoryData.hpp"
#include <QScreen>
#include <QScrollArea>

namespace QHelper
{
    QNotificationShowHistory::QNotificationShowHistory(void)
    {
        _ui.setupUi(this);
        _targetScreenRect = QToastNotification::getInstance().getTargetScreenRect();
        _ui.scrollArea->setVisible(false);
        this->hide();
    }

    void QNotificationShowHistory::showHistory(void)
    {
        if (_ui.scrollArea->isVisible())
        {
            while (_ui.verticalLayout->count() > 1)
            {
                QLayoutItem* item = _ui.verticalLayout->takeAt(1);
                if (item && item->widget())
                {
                    delete item->widget();
                }
                if (item)
                {
                    delete item;
                }
            }
            _ui.scrollArea->setVisible(false);
            this->hide();
        }
        else
        {
            for (size_t i = 0; i < QToastNotification::getInstance().getHistory().size(); ++i)
            {
                QToastNotification::sNotificationInfo data = QToastNotification::getInstance().getHistory().at(i);
                QNotificationHistoryData* dataWidget
                    = new QNotificationHistoryData(data.timeStamp, data.title, data.description, data.severityLevel);

                if (dataWidget)
                {
                    _ui.verticalLayout->addWidget(dataWidget);
                }
            }
            _ui.scrollArea->verticalScrollBar()->setValue(_ui.scrollArea->verticalScrollBar()->maximum());
            _ui.scrollArea->setVisible(true);
            this->show();
            this->raise();
            this->activateWindow();
        }
    }
}  // namespace QHelper