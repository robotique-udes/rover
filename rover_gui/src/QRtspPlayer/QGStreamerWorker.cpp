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

}

void GStreamerWorker::stopPipeline()
{
    cleanupGStreamer();
    emit pipelineStopped();
}
