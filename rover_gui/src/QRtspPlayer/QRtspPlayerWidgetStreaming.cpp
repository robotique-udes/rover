#include "QRtspPlayer/QRtspPlayerWidgetHeader.hpp"
#include "QLoggingMacros.hpp"
#include <QMessageBox>
#include <QDateTime>
#include <QRegularExpression>
#include <gst/video/videooverlay.h>

void RtspPlayerWidget::startStream(const QString& rtspUrl_)
{
    if (rtspUrl_.isEmpty())
    {
        LOG_WARNING_TARGET("RtspPlayer", "Empty RTSP URL provided", this->_widgetId.toUtf8().constData());
        return;
    }

    if (!this->validateRtspUrl(rtspUrl_))
    {
        QMessageBox::warning(this, "Invalid RTSP URL",
                           "The URL format is invalid. Please enter a valid RTSP URL.\n\n"
                           "Format: rtsp://[username:password@]host[:port]/path");
        return;
    }
    
    // Prevent duplicate stream starts in quick succession
    QDateTime currentTime = QDateTime::currentDateTime();
    if (rtspUrl_ == this->_lastStreamUrl && this->_lastStreamTime.isValid() && 
        this->_lastStreamTime.msecsTo(currentTime) < 500)
    {
        return;
    }
    
    this->_lastStreamUrl = rtspUrl_;
    this->_lastStreamTime = currentTime;

    // Reset attempt counter if this is a fresh connection (not a reconnect)
    if (_state != PlayerState::Reconnecting)
    {
        LOG_INFO_TARGET("RtspPlayer", QString("Starting stream: %1").arg(rtspUrl_), this->_widgetId.toUtf8().constData());
        this->_reconnectAttempts = 0;
    }

    // Update state
    this->setPlayerState(PlayerState::Connecting);
    
    // Request pipeline start
    emit this->requestStartStream(rtspUrl_);
}

void RtspPlayerWidget::stopStream(void)
{
    // Stop connection timeout if still waiting
    this->_connectionTimeoutTimer->stop();
    
    // Skip if no active stream or already stopped
    if (_state == PlayerState::NotConnected || _state == PlayerState::Paused)
    {
        return;
    }

    LOG_INFO_TARGET("RtspPlayer", "Stopping stream", this->_widgetId.toUtf8().constData());
    
    // Stop all timers
    this->_frameTimeoutTimer->stop();
    this->_reconnectTimer->stop();
    
    // Disable streaming-dependent buttons
    this->_arucoButton->setEnabled(false);
    this->_screenshotButton->setEnabled(false);
    this->_recordButton->setEnabled(false);
    
    // Request pipeline stop
    emit this->requestStopStream();
    
    // Update state (to Paused if previously connected, otherwise NotConnected)
    if (this->_wasEverConnected)
    {
        this->setPlayerState(PlayerState::Paused);
    }
    else
    {
        this->setPlayerState(PlayerState::NotConnected);
    }
}

void RtspPlayerWidget::onPipelineStarted(GstElement* pipeline_)
{
    if (!pipeline_)
    {
        LOG_ERROR_TARGET("RtspPlayer", "Pipeline creation failed", this->_widgetId.toUtf8().constData());
        this->_connectionTimeoutTimer->stop();
        this->setPlayerState(PlayerState::ConnectionError);
        return;
    }

    this->_pipeline = pipeline_;
    GstElement* videoSink = gst_bin_get_by_interface(GST_BIN(this->_pipeline), GST_TYPE_VIDEO_OVERLAY);
    
    if (!videoSink)
    {
        LOG_ERROR_TARGET("RtspPlayer", "Failed to find VideoOverlay in pipeline", this->_widgetId.toUtf8().constData());
        this->_connectionTimeoutTimer->stop();
        this->setPlayerState(PlayerState::ConnectionError);
        return;
    }

    // Set up video overlay
    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(videoSink), (guintptr)this->_ui.videoWidget->winId());
    gst_object_unref(videoSink);
    
    // Start pipeline playback
    gst_element_set_state(this->_pipeline, GST_STATE_PLAYING);
    LOG_DEBUG_TARGET("RtspPlayer", "Pipeline state set to PLAYING", this->_widgetId.toUtf8().constData());
    
    // Note: We don't update the state to Streaming here - we wait for actual frames
    // in onFrameReceived() before considering the stream active
    
    // Start timeout for initial frame detection
    this->_frameTimeoutTimer->start(2000);
}

void RtspPlayerWidget::onErrorOccurred(const QString& error_)
{
    if (_state == PlayerState::Streaming)
    {
        // Ignore errors while streaming - the frame timeout will handle connection loss
        return;
    }
    
    if (_state == PlayerState::Reconnecting)
    {
        // Just log debug info during reconnect attempts
        LOG_DEBUG_TARGET("RtspPlayer", error_, this->_widgetId.toUtf8().constData());
        
        // Make sure reconnect timer is running
        if (!this->_reconnectTimer->isActive())
        {
            this->_reconnectTimer->start(3000);
        }
    }
    else
    {
        // Log error and attempt reconnection
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