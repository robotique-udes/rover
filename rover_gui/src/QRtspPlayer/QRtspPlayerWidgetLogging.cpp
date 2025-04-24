#include "QRtspPlayer/QRtspPlayerWidgetHeader.hpp"
#include "QLoggingMacros.hpp"
#include <QScrollBar>

void RtspPlayerWidget::onNewLogMessage(const QString& message_, const QString& target_)
{
    if (target_ == this->_widgetId)
    {
        this->_logDisplay->append(message_);
        
        // Auto-scroll to bottom
        QScrollBar* scrollBar = this->_logDisplay->verticalScrollBar();
        scrollBar->setValue(scrollBar->maximum());
    }
}

void RtspPlayerWidget::onToggleDebug(bool checked_)
{
    QLogManager::getInstance().setShowDebug(checked_, this->_widgetId);
}

void RtspPlayerWidget::onToggleInfo(bool checked_)
{
    QLogManager::getInstance().setShowInfo(checked_, this->_widgetId);
}

void RtspPlayerWidget::onToggleWarning(bool checked_)
{
    QLogManager::getInstance().setShowWarning(checked_, this->_widgetId);
}

void RtspPlayerWidget::onToggleError(bool checked_)
{
    QLogManager::getInstance().setShowError(checked_, this->_widgetId);
}

void RtspPlayerWidget::onClearLogs(void)
{
    this->_logDisplay->clear();
    this->_logDisplay->append("Logs cleared for RTSP player " + this->_widgetId);
}