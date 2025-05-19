#include "QGStreamerWorker.hpp"
#include "QLogManager.hpp"
#include <QDebug>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/videooverlay.h>
#include <QUrl>

using namespace LogUtils;

static constexpr int MAX_CONSECUTIVE_ERRORS = 3;
static int consecutive_errors_count = 0;

static void glib_log_handler(const gchar* log_domain_, GLogLevelFlags log_level_, const gchar* message_, gpointer user_data_)
{
    auto* worker = static_cast<GStreamerWorker*>(user_data_);
    QString targetId = worker ? worker->getTargetId() : QString();

    QString domain = log_domain_ ? log_domain_ : "GLib";
    QString msg = QString("%1: %2").arg(domain).arg(message_);

    if (log_level_ & G_LOG_LEVEL_ERROR || log_level_ & G_LOG_LEVEL_CRITICAL)
    {
        UI_LOG_ERROR_RTSP(msg, targetId);
    }
    else if (log_level_ & G_LOG_LEVEL_WARNING)
    {
        UI_LOG_WARNING_RTSP(msg, targetId);
    }
    else if (log_level_ & G_LOG_LEVEL_MESSAGE || log_level_ & G_LOG_LEVEL_INFO)
    {
        UI_LOG_INFO_RTSP(msg, targetId);
    }
    else
    {
        UI_LOG_DEBUG_RTSP(msg, targetId);
    }
}

GstElement* GStreamerWorker::getPipeline()
{
    return _pipeline;
}

static void on_gst_error_message(GstBus* bus_, GstMessage* msg_, gpointer user_data_)
{
    Q_UNUSED(bus_);
    auto* worker = static_cast<GStreamerWorker*>(user_data_);

    GError* err = nullptr;
    gchar* debug = nullptr;
    gst_message_parse_error(msg_, &err, &debug);

    QString errorMsg = QString("%1").arg(err ? err->message : "Unknown Error");

    UI_LOG_DEBUG_RTSP(errorMsg, worker ? worker->getTargetId() : QString());

    if (worker)
    {
        consecutive_errors_count++;

        if (consecutive_errors_count >= MAX_CONSECUTIVE_ERRORS)
        {
            UI_LOG_ERROR_RTSP("Maximum consecutive errors reached, connection failed", worker->getTargetId());
            emit worker->connectionFailed();
            consecutive_errors_count = 0;
        }
        else
        {
            emit worker->errorOccurred(errorMsg);
        }
    }

    if (err)
    {
        g_error_free(err);
    }
    if (debug)
    {
        g_free(debug);
    }
}

static GstFlowReturn on_new_sample(GstElement* sink_, gpointer user_data_)
{
    auto* worker = static_cast<GStreamerWorker*>(user_data_);
    if (!worker)
    {
        return GST_FLOW_OK;
    }

    consecutive_errors_count = 0;

    emit worker->frameReceived();

    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));
    if (sample)
    {
        gst_sample_unref(sample);
    }

    return GST_FLOW_OK;
}

static void on_gst_warning_message(GstBus* bus_, GstMessage* msg_, gpointer user_data_)
{
    Q_UNUSED(bus_);
    auto* worker = static_cast<GStreamerWorker*>(user_data_);

    GError* err = nullptr;
    gchar* debug = nullptr;
    gst_message_parse_warning(msg_, &err, &debug);

    UI_LOG_DEBUG_RTSP(QString("%1").arg(err ? err->message : "Unknown Warning"), worker ? worker->getTargetId() : QString());

    if (err)
    {
        g_error_free(err);
    }
    if (debug)
    {
        g_free(debug);
    }
}

static void on_decodebin_pad_added(GstElement* decodebin_, GstPad* pad_, gpointer user_data_)
{
    Q_UNUSED(decodebin_);
    auto* worker = static_cast<GStreamerWorker*>(user_data_);
    if (!worker)
    {
        UI_LOG_ERROR_RTSP("Invalid worker pointer in pad-added callback", worker ? worker->getTargetId() : QString());
        return;
    }

    GstElement* queue0 = gst_bin_get_by_name(GST_BIN(worker->getPipeline()), "q0");
    if (!queue0)
    {
        UI_LOG_ERROR_RTSP("Failed to find q0 element", worker->getTargetId());
        return;
    }

    GstPad* queueSinkPad = gst_element_get_static_pad(queue0, "sink");
    if (!queueSinkPad)
    {
        UI_LOG_ERROR_RTSP("Failed to get sink pad from q0", worker->getTargetId());
        gst_object_unref(queue0);
        return;
    }

    if (gst_pad_link(pad_, queueSinkPad) != GST_PAD_LINK_OK)
    {
        UI_LOG_ERROR_RTSP("Failed to link decodebin pad to q0 sink pad", worker->getTargetId());
    }

    gst_object_unref(queueSinkPad);
    gst_object_unref(queue0);
}

static gboolean on_any_message(GstBus* bus_, GstMessage* msg_, gpointer user_data_)
{
    Q_UNUSED(bus_);
    auto* worker = static_cast<GStreamerWorker*>(user_data_);
    QString targetId = worker ? worker->getTargetId() : QString();

    gchar* sourceName = nullptr;
    if (GST_IS_OBJECT(GST_MESSAGE_SRC(msg_)))
    {
        sourceName = gst_object_get_name(GST_MESSAGE_SRC(msg_));
    }

    QString source = sourceName ? sourceName : "unknown";
    QString typeStr = GST_MESSAGE_TYPE_NAME(msg_);

    if (GST_MESSAGE_TYPE(msg_) == GST_MESSAGE_ERROR || GST_MESSAGE_TYPE(msg_) == GST_MESSAGE_WARNING
        || GST_MESSAGE_TYPE(msg_) == GST_MESSAGE_INFO)
    {
        GError* err = nullptr;
        gchar* debug = nullptr;

        if (GST_MESSAGE_TYPE(msg_) == GST_MESSAGE_ERROR)
        {
            gst_message_parse_error(msg_, &err, &debug);
            UI_LOG_ERROR_RTSP(QString("%1: %2").arg(source).arg(err ? err->message : "Unknown Error"), targetId);
        }
        else if (GST_MESSAGE_TYPE(msg_) == GST_MESSAGE_WARNING)
        {
            gst_message_parse_warning(msg_, &err, &debug);
            UI_LOG_WARNING_RTSP(QString("%1: %2").arg(source).arg(err ? err->message : "Unknown Warning"), targetId);
        }
        else if (GST_MESSAGE_TYPE(msg_) == GST_MESSAGE_INFO)
        {
            gst_message_parse_info(msg_, &err, &debug);
            UI_LOG_INFO_RTSP(QString("%1: %2").arg(source).arg(err ? err->message : "Unknown Info"), targetId);
        }

        if (debug)
        {
            UI_LOG_DEBUG_RTSP(QString("Debug info: %1").arg(debug), targetId);
            g_free(debug);
        }

        if (err)
        {
            g_error_free(err);
        }
    }

    if (sourceName)
    {
        g_free(sourceName);
    }

    return TRUE;
}

static void my_gst_debug_log_function(GstDebugCategory* category_,
                                      GstDebugLevel level_,
                                      const gchar* file_,
                                      const gchar* function_,
                                      gint line_,
                                      GObject* object_,
                                      GstDebugMessage* message_,
                                      gpointer user_data_)
{
    Q_UNUSED(file_);
    Q_UNUSED(function_);
    Q_UNUSED(line_);
    Q_UNUSED(object_);

    auto* worker = static_cast<GStreamerWorker*>(user_data_);
    QString targetId = worker ? worker->getTargetId() : QString();
    QString msg = gst_debug_message_get(message_);
    QString cat = gst_debug_category_get_name(category_);

    if (level_ <= GST_LEVEL_ERROR)
    {
        UI_LOG_ERROR_RTSP(QString("%1: %2").arg(cat).arg(msg), targetId);
    }
    else if (level_ <= GST_LEVEL_WARNING)
    {
        UI_LOG_WARNING_RTSP(QString("%1: %2").arg(cat).arg(msg), targetId);
    }
    else if (level_ <= GST_LEVEL_INFO)
    {
        UI_LOG_INFO_RTSP(QString("%1: %2").arg(cat).arg(msg), targetId);
    }
    else
    {
        UI_LOG_DEBUG_RTSP(QString("%1: %2").arg(cat).arg(msg), targetId);
    }
}

GStreamerWorker::GStreamerWorker(QObject* parent):
    QObject(parent)
{
    qputenv("GST_DEBUG_NO_COLOR", "1");
    qputenv("GST_DEBUG", "3");

    gst_init(nullptr, nullptr);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
    gst_debug_add_log_function(my_gst_debug_log_function, this, NULL);
    gst_debug_remove_log_function(gst_debug_log_default);
#pragma GCC diagnostic pop

    GLogLevelFlags log_levels = static_cast<GLogLevelFlags>(G_LOG_LEVEL_MASK | G_LOG_FLAG_FATAL | G_LOG_FLAG_RECURSION);

    g_log_set_handler("GLib", log_levels, glib_log_handler, this);
    g_log_set_handler("GLib-GObject", log_levels, glib_log_handler, this);
    g_log_set_handler("GStreamer", log_levels, glib_log_handler, this);
    g_log_set_handler(NULL, log_levels, glib_log_handler, this);

    gst_debug_set_default_threshold(GST_LEVEL_WARNING);
}

GStreamerWorker::~GStreamerWorker()
{
    this->cleanupGStreamer();
}

QString GStreamerWorker::buildPipelineString(const QString& rtspUrl_) const
{
    return QString(
               "rtspsrc location=%1 latency=100 timeout=10000000 buffer-mode=none do-retransmission=false drop-on-latency=true ! "
               "decodebin "
               "name=dec "
               "queue name=q0 max-size-buffers=10 max-size-time=0 max-size-bytes=0 leaky=downstream ! videoconvert ! tee name=t "
               "t. ! queue max-size-buffers=2 leaky=downstream ! videoscale ! video/x-raw,pixel-aspect-ratio=1/1 ! ximagesink "
               "sync=false "
               "t. ! queue max-size-buffers=2 leaky=downstream ! videoconvert ! appsink name=myappsink sync=false")
        .arg(rtspUrl_);
}

void GStreamerWorker::startPipeline(const QString& rtspUrl_)
{
    this->cleanupGStreamer();

    _lastUrl = rtspUrl_;

    QUrl url(rtspUrl_);
    if (!url.isValid() || url.host().isEmpty())
    {
        UI_LOG_ERROR_RTSP("Invalid URL or missing host part", _targetId);
        emit errorOccurred("Invalid URL format");
        return;
    }

    const QString pipelineDesc = this->buildPipelineString(rtspUrl_);
    _pipeline = gst_parse_launch(pipelineDesc.toUtf8().constData(), nullptr);

    if (!_pipeline)
    {
        UI_LOG_ERROR_RTSP("Failed to create pipeline", _targetId);
        emit errorOccurred("Failed to create GStreamer pipeline");
        return;
    }

    GstElement* decodebin = gst_bin_get_by_name(GST_BIN(_pipeline), "dec");
    if (!decodebin)
    {
        UI_LOG_ERROR_RTSP("Failed to get decodebin element", _targetId);
        emit errorOccurred("Failed to get decodebin element from pipeline");
        return;
    }

    g_signal_connect(decodebin, "pad-added", G_CALLBACK(on_decodebin_pad_added), this);
    gst_object_unref(decodebin);

    GstElement* appSink = gst_bin_get_by_name(GST_BIN(_pipeline), "myappsink");
    if (!appSink)
    {
        UI_LOG_ERROR_RTSP("Failed to get appsink", _targetId);
        emit errorOccurred("Failed to get appsink");
        return;
    }

    g_object_set(G_OBJECT(appSink), "emit-signals", TRUE, "sync", FALSE, "max-buffers", 2, "drop", TRUE, nullptr);

    static gulong signal_id = 0;
    if (signal_id != 0)
    {
        g_signal_handler_disconnect(appSink, signal_id);
    }

    if (_newSampleSignalId != 0)
    {
        g_signal_handler_disconnect(appSink, _newSampleSignalId);
        _newSampleSignalId = 0;
    }

    _newSampleSignalId = g_signal_connect(appSink, "new-sample", G_CALLBACK(on_new_sample), this);

    gst_object_unref(appSink);

    GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(_pipeline));
    if (!bus)
    {
        UI_LOG_ERROR_RTSP("Failed to get GStreamer bus", _targetId);
        emit errorOccurred("Failed to get GStreamer bus");
        return;
    }

    gst_bus_add_signal_watch(bus);

    g_signal_connect(bus, "message::error", G_CALLBACK(on_gst_error_message), this);
    g_signal_connect(bus, "message::warning", G_CALLBACK(on_gst_warning_message), this);

    g_signal_connect(bus, "message", G_CALLBACK(on_any_message), this);

    g_object_unref(bus);

    consecutive_errors_count = 0;

    UI_LOG_DEBUG_RTSP("Pipeline started", _targetId);
    emit pipelineStarted(_pipeline);
}

void GStreamerWorker::stopPipeline()
{
    this->cleanupGStreamer();
    emit pipelineStopped();
}

void GStreamerWorker::cleanupGStreamer()
{
    if (_pipeline)
    {
        GstStateChangeReturn ret = gst_element_set_state(_pipeline, GST_STATE_NULL);

        if (ret == GST_STATE_CHANGE_ASYNC)
        {
            gst_element_get_state(_pipeline, NULL, NULL, GST_CLOCK_TIME_NONE);
        }

        GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(_pipeline));
        if (bus)
        {
            gst_bus_remove_signal_watch(bus);
            gst_object_unref(bus);
        }

        gst_object_unref(_pipeline);
        _pipeline = nullptr;
    }
}

void GStreamerWorker::setTargetId(const QString& id_)
{
    _targetId = id_;
}

QString GStreamerWorker::getTargetId(void) const
{
    return _targetId;
}