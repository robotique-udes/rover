#include "QGStreamerWorker.hpp"
#include "QLogManager.hpp"
#include <QDebug>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/videooverlay.h>
#include <QUrl>
#include <rclcpp/logging.hpp>
#include <mutex>

using namespace LogUtils;

static constexpr int MAX_CONSECUTIVE_ERRORS = 3;
static int consecutive_errors_count = 0;

static void glib_log_handler(const gchar* log_domain, GLogLevelFlags log_level, const gchar* message, gpointer /*user_data*/)
{
    static const rclcpp::Logger logger = rclcpp::get_logger("gstreamer");

    const char* domain = log_domain ? log_domain : "GLib";
    const char* msg = message ? message : "";

    if (log_level & (G_LOG_LEVEL_ERROR | G_LOG_LEVEL_CRITICAL))
    {
        RCLCPP_ERROR(logger, "%s: %s", domain, msg);
    }
    else if (log_level & G_LOG_LEVEL_WARNING)
    {
        RCLCPP_WARN(logger, "%s: %s", domain, msg);
    }
    else if (log_level & (G_LOG_LEVEL_MESSAGE | G_LOG_LEVEL_INFO))
    {
        RCLCPP_INFO(logger, "%s: %s", domain, msg);
    }
    else
    {
        RCLCPP_DEBUG(logger, "%s: %s", domain, msg);
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

    UI_LOG_DEBUG_RTSP(errorMsg, worker ? worker->getTargetWidget() : nullptr);

    if (worker)
    {
        consecutive_errors_count++;

        if (consecutive_errors_count >= MAX_CONSECUTIVE_ERRORS)
        {
            UI_LOG_ERROR_RTSP("Maximum consecutive errors reached, connection failed", worker->getTargetWidget());
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

    UI_LOG_DEBUG_RTSP(QString("%1").arg(err ? err->message : "Unknown Warning"), worker ? worker->getTargetWidget() : nullptr);

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
        UI_LOG_ERROR_RTSP("Invalid worker pointer in pad-added callback", worker ? worker->getTargetWidget() : nullptr);
        return;
    }

    GstElement* queue0 = gst_bin_get_by_name(GST_BIN(worker->getPipeline()), "q0");
    if (!queue0)
    {
        UI_LOG_ERROR_RTSP("Failed to find q0 element", worker->getTargetWidget());
        return;
    }

    GstPad* queueSinkPad = gst_element_get_static_pad(queue0, "sink");
    if (!queueSinkPad)
    {
        UI_LOG_ERROR_RTSP("Failed to get sink pad from q0", worker->getTargetWidget());
        gst_object_unref(queue0);
        return;
    }

    if (gst_pad_link(pad_, queueSinkPad) != GST_PAD_LINK_OK)
    {
        UI_LOG_ERROR_RTSP("Failed to link decodebin pad to q0 sink pad", worker->getTargetWidget());
    }

    gst_object_unref(queueSinkPad);
    gst_object_unref(queue0);
}

static gboolean on_any_message(GstBus* bus_, GstMessage* msg_, gpointer user_data_)
{
    Q_UNUSED(bus_);
    auto* worker = static_cast<GStreamerWorker*>(user_data_);
    QWidget* targetWidget = worker ? worker->getTargetWidget() : nullptr;

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
            UI_LOG_ERROR_RTSP(QString("%1: %2").arg(source).arg(err ? err->message : "Unknown Error"), targetWidget);
        }
        else if (GST_MESSAGE_TYPE(msg_) == GST_MESSAGE_WARNING)
        {
            gst_message_parse_warning(msg_, &err, &debug);
            UI_LOG_WARNING_RTSP(QString("%1: %2").arg(source).arg(err ? err->message : "Unknown Warning"), targetWidget);
        }
        else if (GST_MESSAGE_TYPE(msg_) == GST_MESSAGE_INFO)
        {
            gst_message_parse_info(msg_, &err, &debug);
            UI_LOG_INFO_RTSP(QString("%1: %2").arg(source).arg(err ? err->message : "Unknown Info"), targetWidget);
        }

        if (debug)
        {
            UI_LOG_DEBUG_RTSP(QString("Debug info: %1").arg(debug), targetWidget);
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

static void my_gst_debug_log_function(GstDebugCategory* category,
                                      GstDebugLevel level,
                                      const gchar* /*file*/,
                                      const gchar* /*function*/,
                                      gint /*line*/,
                                      GObject* /*object*/,
                                      GstDebugMessage* message,
                                      gpointer /*user_data*/)
{
    static const rclcpp::Logger logger = rclcpp::get_logger("gstreamer");

    const char* cat = gst_debug_category_get_name(category);
    const char* msg = gst_debug_message_get(message);
    if (!msg)
    {
        msg = "";
    }

    if (level <= GST_LEVEL_ERROR)
    {
        RCLCPP_ERROR(logger, "%s: %s", cat, msg);
    }
    else if (level <= GST_LEVEL_WARNING)
    {
        RCLCPP_WARN(logger, "%s: %s", cat, msg);
    }
    else if (level <= GST_LEVEL_INFO)
    {
        RCLCPP_INFO(logger, "%s: %s", cat, msg);
    }
    else
    {
        RCLCPP_DEBUG(logger, "%s: %s", cat, msg);
    }
}

GStreamerWorker::GStreamerWorker(QObject* parent):
    QObject(parent)
{
    static std::once_flag once;
    std::call_once(once,
                   []
                   {
                       qputenv("GST_DEBUG_NO_COLOR", "1");  // must come before gst_init
                       qputenv("GST_DEBUG", "3");
                       gst_init(nullptr, nullptr);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
                       gst_debug_remove_log_function(gst_debug_log_default);
                       gst_debug_add_log_function(my_gst_debug_log_function, nullptr, nullptr);
#pragma GCC diagnostic pop

                       const GLogLevelFlags levels
                           = static_cast<GLogLevelFlags>(G_LOG_LEVEL_MASK | G_LOG_FLAG_FATAL | G_LOG_FLAG_RECURSION);
                       for (const char* domain : {"GLib", "GLib-GObject", "GStreamer", (const char*)nullptr})
                       {
                           g_log_set_handler(domain, levels, glib_log_handler, nullptr);
                       }

                       gst_debug_set_default_threshold(GST_LEVEL_WARNING);
                   });
}

GStreamerWorker::~GStreamerWorker()
{
    this->cleanupGStreamer();
}

QString GStreamerWorker::buildPipelineString(const QString& rtspUrl_) const
{
    return QString(
               "rtspsrc location=%1 latency=0 protocols=tcp timeout=10000000 buffer-mode=none do-retransmission=false "
               "drop-on-latency=true ! "
               "decodebin "
               "name=dec "
               "queue name=q0 max-size-buffers=10 max-size-time=0 max-size-bytes=0 leaky=downstream ! videoconvert ! tee name=t "
               "t. ! queue max-size-buffers=2 leaky=downstream ! videoscale ! video/x-raw,pixel-aspect-ratio=1/1 ! ximagesink "
               "sync=false "
               "t. ! queue max-size-buffers=2 leaky=downstream ! videoconvert ! appsink name=myappsink sync=true")
        .arg(rtspUrl_);
}

void GStreamerWorker::startPipeline(const QString& rtspUrl_)
{
    this->cleanupGStreamer();

    _lastUrl = rtspUrl_;

    QUrl url(rtspUrl_);
    if (!url.isValid() || url.host().isEmpty())
    {
        UI_LOG_ERROR_RTSP("Invalid URL or missing host part", _targetWidget);
        emit errorOccurred("Invalid URL format");
        return;
    }

    const QString pipelineDesc = this->buildPipelineString(rtspUrl_);
    _pipeline = gst_parse_launch(pipelineDesc.toUtf8().constData(), nullptr);

    if (!_pipeline)
    {
        UI_LOG_ERROR_RTSP("Failed to create pipeline", _targetWidget);
        emit errorOccurred("Failed to create GStreamer pipeline");
        return;
    }

    GstElement* decodebin = gst_bin_get_by_name(GST_BIN(_pipeline), "dec");
    if (!decodebin)
    {
        UI_LOG_ERROR_RTSP("Failed to get decodebin element", _targetWidget);
        emit errorOccurred("Failed to get decodebin element from pipeline");
        return;
    }

    g_signal_connect(decodebin, "pad-added", G_CALLBACK(on_decodebin_pad_added), this);
    gst_object_unref(decodebin);

    GstElement* appSink = gst_bin_get_by_name(GST_BIN(_pipeline), "myappsink");
    if (!appSink)
    {
        UI_LOG_ERROR_RTSP("Failed to get appsink", _targetWidget);
        emit errorOccurred("Failed to get appsink");
        return;
    }

    g_object_set(G_OBJECT(appSink), "emit-signals", TRUE, "sync", FALSE, "max-buffers", 2, "drop", TRUE, nullptr);

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
        UI_LOG_ERROR_RTSP("Failed to get GStreamer bus", _targetWidget);
        emit errorOccurred("Failed to get GStreamer bus");
        return;
    }

    gst_bus_add_signal_watch(bus);

    g_signal_connect(bus, "message::error", G_CALLBACK(on_gst_error_message), this);
    g_signal_connect(bus, "message::warning", G_CALLBACK(on_gst_warning_message), this);

    g_signal_connect(bus, "message", G_CALLBACK(on_any_message), this);

    g_object_unref(bus);

    consecutive_errors_count = 0;

    UI_LOG_DEBUG_RTSP("Pipeline started", _targetWidget);
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

void GStreamerWorker::setTargetWidget(QWidget* widget)
{
    _targetWidget = widget;
}

QWidget* GStreamerWorker::getTargetWidget() const
{
    return _targetWidget;
}
