#include "QGStreamerWorker.hpp"
#include <gst/gst.h>
#include <gst/video/videooverlay.h>

GStreamerWorker::GStreamerWorker(QObject* parent)
    : QObject(parent), pipeline(nullptr)
{
    gst_init(nullptr, nullptr);
}

GStreamerWorker::~GStreamerWorker()
{
    cleanupGStreamer();
}

void GStreamerWorker::initializeGStreamer()
{
    // Initialization if needed
}

void GStreamerWorker::cleanupGStreamer()
{
    if (pipeline) {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
        pipeline = nullptr;
    }
}

void GStreamerWorker::startPipeline(const QString& rtspUrl)
{
    cleanupGStreamer();

    pipeline = gst_parse_launch(
        QString("rtspsrc location=%1 latency=50 ! decodebin ! videoconvert ! xvimagesink sync=false").arg(rtspUrl).toUtf8().constData(), nullptr);

    if (!pipeline) {
        emit errorOccurred("Failed to create GStreamer pipeline.");
        return;
    }

    emit pipelineStarted(pipeline);

    /*GstElement* videoSink = gst_bin_get_by_interface(GST_BIN(pipeline), GST_TYPE_VIDEO_OVERLAY);
    if (videoSink) {
        // Force creation of a native window for videoWidget
        ui->videoWidget->winId();
        
        // Set the window handle for the overlay
        gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(videoSink), (guintptr)ui->videoWidget->winId());
        
        emit pipelineStarted();
    } else {
        emit errorOccurred("Failed to configure video sink.");
        return;
    }

    gst_element_set_state(pipeline, GST_STATE_PLAYING);*/
}

void GStreamerWorker::stopPipeline()
{
    cleanupGStreamer();
    emit pipelineStopped();
}
