#include "QGStreamerWorker.hpp"
#include <QDebug>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/videooverlay.h>
#include <QUrl>
#include <algorithm>
#include <cctype>

static rclcpp::Logger gst_logger = rclcpp::get_logger("VIDEOPLAYER");

void GStreamerWorker::on_gst_error_message(GstBus* bus_, GstMessage* msg_, gpointer user_data_)
{
    Q_UNUSED(bus_);

    auto* worker = static_cast<GStreamerWorker*>(user_data_);
    if (!worker)
        return;

    GError* err = nullptr;
    gchar* debug = nullptr;
    gst_message_parse_error(msg_, &err, &debug);

    std::string errorMsg = err ? err->message : "Unknown Error";
    RCLCPP_ERROR(gst_logger, "GStreamer error: %s", errorMsg.c_str());

    emit worker->errorOccurred(QString::fromStdString(errorMsg));

    if (err)
        g_error_free(err);
    if (debug)
        g_free(debug);
}

GstFlowReturn GStreamerWorker::on_new_sample(GstElement* sink_, gpointer user_data_)
{
    auto* worker = static_cast<GStreamerWorker*>(user_data_);
    if (!worker)
        return GST_FLOW_OK;

    emit worker->frameReceived();

    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));
    if (sample)
    {
        gst_sample_unref(sample);
    }

    return GST_FLOW_OK;
}

static void on_decodebin_pad_added(GstElement* decodebin_, GstPad* pad_, gpointer user_data_)
{
    Q_UNUSED(user_data_);

    GstElement* pipeline = GST_ELEMENT(gst_element_get_parent(decodebin_));
    if (!pipeline)
        return;

    GstElement* queue0 = gst_bin_get_by_name(GST_BIN(pipeline), "q0");
    gst_object_unref(pipeline);

    if (!queue0)
        return;

    GstPad* queueSinkPad = gst_element_get_static_pad(queue0, "sink");
    if (!queueSinkPad)
    {
        gst_object_unref(queue0);
        return;
    }

    gst_pad_link(pad_, queueSinkPad);

    gst_object_unref(queueSinkPad);
    gst_object_unref(queue0);
}

static void minimal_gst_debug_function(GstDebugCategory* category_,
                                       GstDebugLevel level_,
                                       const gchar* file_,
                                       const gchar* function_,
                                       gint line_,
                                       GObject* object_,
                                       GstDebugMessage* message_,
                                       gpointer user_data_)
{
    (void)file_;
    (void)function_;
    (void)line_;
    (void)object_;
    (void)user_data_;

    if (level_ <= GST_LEVEL_ERROR)
    {
        const gchar* msg = gst_debug_message_get(message_);
        const gchar* cat = gst_debug_category_get_name(category_);

        std::string msgStr(msg);
        std::transform(msgStr.begin(), msgStr.end(), msgStr.begin(), ::tolower);

        if (msgStr.find("qos") == std::string::npos && msgStr.find("latency") == std::string::npos)
        {
            RCLCPP_ERROR(gst_logger, "[%s] %s", cat, msg);
        }
    }
}

GStreamerWorker::GStreamerWorker(QObject* parent):
    QObject(parent)
{
    gst_init(nullptr, nullptr);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
    gst_debug_add_log_function(minimal_gst_debug_function, this, NULL);
    gst_debug_remove_log_function(gst_debug_log_default);
#pragma GCC diagnostic pop
}

GStreamerWorker::~GStreamerWorker()
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
    gst_debug_remove_log_function(minimal_gst_debug_function);
#pragma GCC diagnostic pop
    this->cleanupGStreamer();
}

std::string GStreamerWorker::buildPipelineString(const std::string& rtspUrl_) const
{
    return "rtspsrc location=" + rtspUrl_
           + " latency=100 timeout=10000000 buffer-mode=none do-retransmission=false drop-on-latency=true "
             "tcp-timeout=20000000 connection-speed=1000 protocols=tcp ! "
             "decodebin "
             "name=dec "
             "queue name=q0 max-size-buffers=10 max-size-time=0 max-size-bytes=0 leaky=downstream ! videoconvert ! tee name=t "
             "t. ! queue max-size-buffers=2 leaky=downstream ! videoscale ! video/x-raw,pixel-aspect-ratio=1/1 ! ximagesink "
             "sync=false "
             "t. ! queue max-size-buffers=2 leaky=downstream ! videoconvert ! appsink name=myappsink sync=false";
}

void GStreamerWorker::startPipeline(const QString& rtspUrl_)
{
    this->cleanupGStreamer();

    _lastUrl = rtspUrl_;

    std::string urlStr = rtspUrl_.toStdString();
    const std::string pipelineDesc = this->buildPipelineString(urlStr);

    GstElement* new_pipeline = gst_parse_launch(pipelineDesc.c_str(), nullptr);

    if (!new_pipeline)
    {
        RCLCPP_ERROR(gst_logger, "Failed to create pipeline");
        emit errorOccurred("Failed to create GStreamer pipeline");
        return;
    }

    GstElement* decodebin = gst_bin_get_by_name(GST_BIN(new_pipeline), "dec");
    if (!decodebin)
    {
        RCLCPP_ERROR(gst_logger, "Failed to get decodebin element");
        emit errorOccurred("Failed to get decodebin element from pipeline");
        gst_object_unref(new_pipeline);
        return;
    }

    g_signal_connect(decodebin, "pad-added", G_CALLBACK(on_decodebin_pad_added), nullptr);
    gst_object_unref(decodebin);

    GstElement* appSink = gst_bin_get_by_name(GST_BIN(new_pipeline), "myappsink");
    if (!appSink)
    {
        RCLCPP_ERROR(gst_logger, "Failed to get appsink");
        emit errorOccurred("Failed to get appsink");
        gst_object_unref(new_pipeline);
        return;
    }

    g_object_set(G_OBJECT(appSink), "emit-signals", TRUE, "sync", FALSE, "max-buffers", 2, "drop", TRUE, nullptr);

    _newSampleSignalId = g_signal_connect(appSink, "new-sample", G_CALLBACK(GStreamerWorker::on_new_sample), this);

    gst_object_unref(appSink);

    GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(new_pipeline));
    if (!bus)
    {
        RCLCPP_ERROR(gst_logger, "Failed to get GStreamer bus");
        emit errorOccurred("Failed to get GStreamer bus");
        gst_object_unref(new_pipeline);
        return;
    }

    gst_bus_add_signal_watch(bus);

    _errorHandlerId = g_signal_connect(bus, "message::error", G_CALLBACK(GStreamerWorker::on_gst_error_message), this);

    g_object_unref(bus);

    GstStateChangeReturn ret = gst_element_set_state(new_pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        RCLCPP_ERROR(gst_logger, "Failed to start pipeline");
        emit errorOccurred("Failed to start GStreamer pipeline");
        gst_object_unref(new_pipeline);
        return;
    }

    _pipeline = new_pipeline;

    emit pipelineStarted(_pipeline);
}

void GStreamerWorker::pausePipeline()
{
    if (!_pipeline)
        return;

    GstStateChangeReturn ret = gst_element_set_state(_pipeline, GST_STATE_PAUSED);
    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        RCLCPP_ERROR(gst_logger, "Failed to pause pipeline");
        emit errorOccurred("Failed to pause pipeline");
    }
}

void GStreamerWorker::stopPipeline()
{
    this->cleanupGStreamer();
}

void GStreamerWorker::cleanupGStreamer()
{
    if (!_pipeline)
        return;

    GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(_pipeline));
    if (bus)
    {
        if (_errorHandlerId)
        {
            g_signal_handler_disconnect(bus, _errorHandlerId);
            _errorHandlerId = 0;
        }
        gst_bus_remove_signal_watch(bus);
        gst_object_unref(bus);
    }

    GstElement* appSink = gst_bin_get_by_name(GST_BIN(_pipeline), "myappsink");
    if (appSink)
    {
        if (_newSampleSignalId != 0)
        {
            g_signal_handler_disconnect(appSink, _newSampleSignalId);
            _newSampleSignalId = 0;
        }
        gst_object_unref(appSink);
    }

    gst_element_set_state(_pipeline, GST_STATE_NULL);
    gst_element_get_state(_pipeline, NULL, NULL, GST_CLOCK_TIME_NONE);
    gst_object_unref(_pipeline);
    _pipeline = nullptr;
}

void GStreamerWorker::setTargetWidget(QWidget* widget)
{
    _targetWidget = widget;
}