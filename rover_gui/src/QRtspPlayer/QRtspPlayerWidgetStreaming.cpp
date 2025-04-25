#include "QRtspPlayer/QRtspPlayerWidgetHeader.hpp"
#include "QLoggingMacros.hpp"
#include <QMessageBox>
#include <QDateTime>
#include <QRegularExpression>
#include <gst/video/videooverlay.h>

void RtspPlayerWidget::onPipelineStarted(GstElement* pipeline_)
{
    if (!pipeline_)
    {
        LOG_ERROR_TARGET("RtspPlayer", "Pipeline creation failed", this->_widgetId.toUtf8().constData());
        this->_connectionTimeoutTimer.stop();
        this->setPlayerState(PlayerState::ConnectionError);
        return;
    }

    this->_pipeline = pipeline_;
    GstElement* videoSink = gst_bin_get_by_interface(GST_BIN(this->_pipeline), GST_TYPE_VIDEO_OVERLAY);
    
    if (!videoSink)
    {
        LOG_ERROR_TARGET("RtspPlayer", "Failed to find VideoOverlay in pipeline", this->_widgetId.toUtf8().constData());
        this->_connectionTimeoutTimer.stop();
        this->setPlayerState(PlayerState::ConnectionError);
        return;
    }

    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(videoSink), (guintptr)this->_ui.videoWidget->winId());
    gst_object_unref(videoSink);
    
    gst_element_set_state(this->_pipeline, GST_STATE_PLAYING);
    LOG_DEBUG_TARGET("RtspPlayer", "Pipeline state set to PLAYING", this->_widgetId.toUtf8().constData());
    
    this->_frameTimeoutTimer.start(2000);
}

void RtspPlayerWidget::onErrorOccurred(const QString& error_)
{
    if (_state == PlayerState::Streaming)
    {
        return;
    }
    
    if (_state == PlayerState::Reconnecting)
    {
        LOG_DEBUG_TARGET("RtspPlayer", error_, this->_widgetId.toUtf8().constData());
        
        if (!this->_reconnectTimer.isActive())
        {
            this->_reconnectTimer.start(3000);
        }
    }
    else
    {
        LOG_ERROR_TARGET("RtspPlayer", error_, this->_widgetId.toUtf8().constData());
        this->setPlayerState(PlayerState::ConnectionError);
    }
}

bool RtspPlayerWidget::validateRtspUrl(const QString& url_)
{
    // Regular expression to validate RTSP URLs
    static QRegularExpression rtspRegex(
        "^rtsp://(?:([^:@]+)(?::([^@]+))?@)?([^:/]+)(?::(\\d+))?(/.*)?$",
        QRegularExpression::CaseInsensitiveOption
    );
    
    QRegularExpressionMatch match = rtspRegex.match(url_);
    if (!match.hasMatch())
    {
        LOG_WARNING_TARGET("RtspPlayer", "Invalid RTSP URL format: " + url_, this->_widgetId.toUtf8().constData());
        return false;
    }
    
    LOG_DEBUG_TARGET("RtspPlayer", "Valid RTSP URL: " + url_, this->_widgetId.toUtf8().constData());
    return true;
}

void RtspPlayerWidget::updateUrlValidationUI(bool isValid_)
{
    if (isValid_)
    {
        this->_ui.rtspUrlInput->setStyleSheet("QLineEdit { border: 1px solid #5cb85c; }");
        this->_ui.rtspUrlInput->setToolTip("Valid RTSP URL");
    }
    else
    {
        this->_ui.rtspUrlInput->setStyleSheet("QLineEdit { border: 1px solid #d9534f; }");
        this->_ui.rtspUrlInput->setToolTip("Invalid RTSP URL format.\nExpected: rtsp://[username:password@]host[:port]/path");
    }
}