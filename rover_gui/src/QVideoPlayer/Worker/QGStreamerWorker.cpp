#include "QGStreamerWorker.hpp"
#include <QDebug>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/videooverlay.h>
#include <QUrl>
#include <algorithm>
#include <cctype>

namespace
{

    constexpr int RTSP_LATENCY_MS = 0;
    constexpr int RTSP_TIMEOUT_MICROSECONDS = 10000000;
    constexpr int RTSP_CONNECTION_SPEED_KBPS = 1000;

    constexpr int QUEUE_MAX_BUFFERS = 1;
    constexpr int QUEUE_MAX_TIME_NS = 0;
    constexpr int QUEUE_MAX_BYTES = 0;

    constexpr int APPSINK_MAX_BUFFERS = 2;
    constexpr bool APPSINK_EMIT_SIGNALS = true;
    constexpr bool APPSINK_SYNC_DISABLED = false;
    constexpr bool APPSINK_DROP_ENABLED = true;

    constexpr const char* DECODEBIN_NAME = "dec";
    constexpr const char* QUEUE_NAME = "q0";
    constexpr const char* APPSINK_NAME = "myappsink";
    constexpr const char* TEE_NAME = "t";

    constexpr const char* EMIT_SIGNALS_PROPERTY = "emit-signals";
    constexpr const char* SYNC_PROPERTY = "sync";
    constexpr const char* MAX_BUFFERS_PROPERTY = "max-buffers";
    constexpr const char* DROP_PROPERTY = "drop";

    constexpr gulong INVALID_SIGNAL_ID = 0;

    const std::string QOS_FILTER = "qos";
    const std::string LATENCY_FILTER = "latency";
}  // namespace

void GStreamerWorker::on_gst_error_message(GstBus* /*bus_*/, GstMessage* message_, gpointer user_data_)
{
    auto* worker = static_cast<GStreamerWorker*>(user_data_);
    if (!worker)
        return;

    GError* gstreamer_error = nullptr;
    gchar* debug_info = nullptr;
    gst_message_parse_error(message_, &gstreamer_error, &debug_info);

    std::string errorMessage = gstreamer_error ? gstreamer_error->message : "Unknown GStreamer Error";
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("GUI"), "GStreamer error:" << errorMessage.c_str());

    emit worker->errorOccurred(QString::fromStdString(errorMessage));

    if (gstreamer_error)
    {
        g_error_free(gstreamer_error);
    }
    if (debug_info)
    {
        g_free(debug_info);
    }
}

GstFlowReturn GStreamerWorker::on_new_sample(GstElement* sink_, gpointer user_data_)
{
    auto* worker = static_cast<GStreamerWorker*>(user_data_);
    if (!worker)
        return GST_FLOW_OK;

    emit worker->frameReceived();

    GstSample* video_sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));
    if (video_sample)
    {
        gst_sample_unref(video_sample);
    }

    return GST_FLOW_OK;
}

static void on_decodebin_pad_added(GstElement* decodebin_, GstPad* new_pad_, gpointer /*user_data_*/)
{
    GstElement* pipeline = GST_ELEMENT(gst_element_get_parent(decodebin_));
    if (!pipeline)
    {
        return;
    }

    GstElement* target_queue = gst_bin_get_by_name(GST_BIN(pipeline), QUEUE_NAME);
    gst_object_unref(pipeline);

    if (!target_queue)
    {
        return;
    }

    GstPad* queue_sink_pad = gst_element_get_static_pad(target_queue, "sink");
    if (!queue_sink_pad)
    {
        gst_object_unref(target_queue);
        return;
    }

    gst_pad_link(new_pad_, queue_sink_pad);

    gst_object_unref(queue_sink_pad);
    gst_object_unref(target_queue);
}

static void minimal_gst_debug_function(GstDebugCategory* debug_category_,
                                       GstDebugLevel debug_level_,
                                       const gchar* /*source_file_*/,
                                       const gchar* /*function_name_*/,
                                       gint /*line_number_*/,
                                       GObject* /*gst_object_*/,
                                       GstDebugMessage* debug_message_,
                                       gpointer /*user_data_*/)
{
    if (debug_level_ <= GST_LEVEL_ERROR)
    {
        const gchar* message_text = gst_debug_message_get(debug_message_);
        const gchar* category_name = gst_debug_category_get_name(debug_category_);

        std::string lowercase_message(message_text);
        std::transform(lowercase_message.begin(), lowercase_message.end(), lowercase_message.begin(), ::tolower);

        // Filter out QoS and latency messages to reduce noise
        if (lowercase_message.find(QOS_FILTER) == std::string::npos
            && lowercase_message.find(LATENCY_FILTER) == std::string::npos)
        {
            RCLCPP_ERROR(rclcpp::get_logger("GUI"), "[%s] %s", category_name, message_text);
        }
    }
}

GStreamerWorker::GStreamerWorker(QObject* parent):
    QObject(parent)
{
    gst_init(nullptr, nullptr);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
    gst_debug_add_log_function(minimal_gst_debug_function, this, nullptr);
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

std::string GStreamerWorker::buildPipelineString(const std::string& rtsp_url_) const
{
    return "rtspsrc location=" + rtsp_url_ + " latency=" + std::to_string(RTSP_LATENCY_MS) + " timeout="
           + std::to_string(RTSP_TIMEOUT_MICROSECONDS) + " buffer-mode=none do-retransmission=false drop-on-latency=true"
           + " connection-speed=" + std::to_string(RTSP_CONNECTION_SPEED_KBPS)
           + " protocols=udp ! "
             "decodebin name="
           + DECODEBIN_NAME
           + " "
             "queue name="
           + QUEUE_NAME + " max-size-buffers=" + std::to_string(QUEUE_MAX_BUFFERS)
           + " max-size-time=" + std::to_string(QUEUE_MAX_TIME_NS) + " max-size-bytes=" + std::to_string(QUEUE_MAX_BYTES)
           + " leaky=downstream ! videoconvert ! tee name=" + TEE_NAME + " " + TEE_NAME
           + ". ! queue max-size-buffers=" + std::to_string(QUEUE_MAX_BUFFERS)
           + " leaky=downstream ! videoscale ! video/x-raw,pixel-aspect-ratio=1/1 ! ximagesink sync=false " + TEE_NAME
           + ". ! queue max-size-buffers=" + std::to_string(QUEUE_MAX_BUFFERS)
           + " leaky=downstream ! videoconvert ! appsink name=" + APPSINK_NAME + " sync=false";
}

void GStreamerWorker::startPipeline(const QString& rtsp_url_)
{
    this->cleanupGStreamer();

    _last_rtsp_url = rtsp_url_;

    std::string url_string = rtsp_url_.toStdString();
    const std::string pipeline_description = this->buildPipelineString(url_string);

    GstElement* new_pipeline = gst_parse_launch(pipeline_description.c_str(), nullptr);

    if (!new_pipeline)
    {
        emit this->errorOccurred("Failed to create GStreamer pipeline");
        return;
    }

    GstElement* decodebin_element = gst_bin_get_by_name(GST_BIN(new_pipeline), DECODEBIN_NAME);
    if (!decodebin_element)
    {
        emit this->errorOccurred("Failed to get decodebin element from pipeline");
        gst_object_unref(new_pipeline);
        return;
    }

    g_signal_connect(decodebin_element, "pad-added", G_CALLBACK(on_decodebin_pad_added), nullptr);
    gst_object_unref(decodebin_element);

    GstElement* app_sink_element = gst_bin_get_by_name(GST_BIN(new_pipeline), APPSINK_NAME);
    if (!app_sink_element)
    {
        emit this->errorOccurred("Failed to get appsink");
        gst_object_unref(new_pipeline);
        return;
    }

    g_object_set(G_OBJECT(app_sink_element),
                 EMIT_SIGNALS_PROPERTY,
                 APPSINK_EMIT_SIGNALS,
                 SYNC_PROPERTY,
                 APPSINK_SYNC_DISABLED,
                 MAX_BUFFERS_PROPERTY,
                 APPSINK_MAX_BUFFERS,
                 DROP_PROPERTY,
                 APPSINK_DROP_ENABLED,
                 nullptr);

    _new_sample_signal_id = g_signal_connect(app_sink_element, "new-sample", G_CALLBACK(GStreamerWorker::on_new_sample), this);

    gst_object_unref(app_sink_element);

    GstBus* pipeline_bus = gst_pipeline_get_bus(GST_PIPELINE(new_pipeline));
    if (!pipeline_bus)
    {
        emit errorOccurred("Failed to get GStreamer bus");
        gst_object_unref(new_pipeline);
        return;
    }

    gst_bus_add_signal_watch(pipeline_bus);

    _error_handler_id = g_signal_connect(pipeline_bus, "message::error", G_CALLBACK(GStreamerWorker::on_gst_error_message), this);

    g_object_unref(pipeline_bus);

    _pipeline = new_pipeline;

    this->setupVideoOverlay();

    GstStateChangeReturn state_change_result = gst_element_set_state(new_pipeline, GST_STATE_PLAYING);
    if (state_change_result == GST_STATE_CHANGE_FAILURE)
    {
        emit this->errorOccurred("Failed to start GStreamer pipeline");
        _pipeline = nullptr;
        gst_object_unref(new_pipeline);
        return;
    }

    emit pipelineStarted(_pipeline);
}

void GStreamerWorker::pausePipeline()
{
    if (!_pipeline)
    {
        return;
    }

    GstStateChangeReturn state_change_result = gst_element_set_state(_pipeline, GST_STATE_PAUSED);
    if (state_change_result == GST_STATE_CHANGE_FAILURE)
    {
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
    {
        return;
    }

    GstBus* pipeline_bus = gst_pipeline_get_bus(GST_PIPELINE(_pipeline));
    if (pipeline_bus)
    {
        if (_error_handler_id != INVALID_SIGNAL_ID)
        {
            g_signal_handler_disconnect(pipeline_bus, _error_handler_id);
            _error_handler_id = INVALID_SIGNAL_ID;
        }
        gst_bus_remove_signal_watch(pipeline_bus);
        gst_object_unref(pipeline_bus);
    }

    GstElement* app_sink_element = gst_bin_get_by_name(GST_BIN(_pipeline), APPSINK_NAME);
    if (app_sink_element)
    {
        if (_new_sample_signal_id != INVALID_SIGNAL_ID)
        {
            g_signal_handler_disconnect(app_sink_element, _new_sample_signal_id);
            _new_sample_signal_id = INVALID_SIGNAL_ID;
        }
        gst_object_unref(app_sink_element);
    }

    gst_element_set_state(_pipeline, GST_STATE_NULL);
    gst_element_get_state(_pipeline, nullptr, nullptr, GST_CLOCK_TIME_NONE);
    gst_object_unref(_pipeline);
    _pipeline = nullptr;
}

void GStreamerWorker::setTargetWidget(QWidget* target_widget_)
{
    _target_widget = target_widget_;
}

void GStreamerWorker::setVideoWidget(QWidget* video_widget_)
{
    _video_widget = video_widget_;
}

void GStreamerWorker::setupVideoOverlay()
{
    if (!_pipeline || !_video_widget)
        return;

    GstElement* video_sink_element = gst_bin_get_by_interface(GST_BIN(_pipeline), GST_TYPE_VIDEO_OVERLAY);
    if (!video_sink_element)
    {
        emit errorOccurred("Failed to get video overlay interface");
        return;
    }

    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(video_sink_element), (guintptr)_video_widget->winId());
    gst_object_unref(video_sink_element);
}