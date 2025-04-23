#include "QRtspPlayer/QRtspPlayerWidgetHeader.hpp"
#include "QLoggingMacros.hpp"
#include <QScrollBar>

void RtspPlayerWidget::onNewLogMessage(const QString& message_, const QString& target_)
{
    // Only process messages for this widget
    if (target_ == this->_widgetId)
    {
        // Add message to log display
        this->_logDisplay->append(message_);
        
        // Auto-scroll to bottom
        QScrollBar* scrollBar = this->_logDisplay->verticalScrollBar();
        scrollBar->setValue(scrollBar->maximum());
    }
}

void RtspPlayerWidget::onToggleDebug(bool checked_)
{
    // Set debug log visibility for this widget's logs
    QLogManager::getInstance().setShowDebug(checked_, this->_widgetId);
}

void RtspPlayerWidget::onToggleInfo(bool checked_)
{
    // Set info log visibility for this widget's logs
    QLogManager::getInstance().setShowInfo(checked_, this->_widgetId);
}

void RtspPlayerWidget::onToggleWarning(bool checked_)
{
    // Set warning log visibility for this widget's logs
    QLogManager::getInstance().setShowWarning(checked_, this->_widgetId);
}

void RtspPlayerWidget::onToggleError(bool checked_)
{
    // Set error log visibility for this widget's logs
    QLogManager::getInstance().setShowError(checked_, this->_widgetId);
}

void RtspPlayerWidget::onClearLogs(void)
{
    // Clear the log display
    this->_logDisplay->clear();
    
    // Add initial message
    this->_logDisplay->append("Logs cleared for RTSP player " + this->_widgetId);
}