#include "QGStreamerWorker.hpp"
#include <QDebug>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/videooverlay.h>
#include <QUrl>

static rclcpp::Logger gst_logger = rclcpp::get_logger("VIDEOPLAYER");

GstElement* GStreamerWorker::getPipeline()
{
    std::lock_guard<std::mutex> lock(_pipelineMutex);
    return _pipeline;
}

void GStreamerWorker::on_gst_error_message(GstBus* bus_, GstMessage* msg_, gpointer user_data_)
{
    Q_UNUSED(bus_);
    auto* worker = static_cast<GStreamerWorker*>(user_data_);
    if (!worker)
        return;

    {
        std::lock_guard<std::mutex> lock(worker->_pipelineMutex);
        if (!worker->_pipeline)
            return;
    }

    GError* err = nullptr;
    gchar* debug = nullptr;
    gst_message_parse_error(msg_, &err, &debug);

    worker->_consecutiveErrorsCount++;

    if (worker->_consecutiveErrorsCount >= worker->MAX_CONSECUTIVE_ERRORS)
    {
        RCLCPP_ERROR(gst_logger, "Maximum consecutive errors reached, connection failed");
        emit worker->connectionFailed();
        worker->_consecutiveErrorsCount = 0;
    }
    else
    {
        emit worker->errorOccurred(errorMsg);
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

GstFlowReturn GStreamerWorker::on_new_sample(GstElement* sink_, gpointer user_data_)
{
    auto* worker = static_cast<GStreamerWorker*>(user_data_);
    if (!worker)
    {
        return GST_FLOW_OK;
    }

    worker->_consecutiveErrorsCount = 0;

    emit worker->frameReceived();

    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));
    if (sample)
    {
        gst_sample_unref(sample);
    }

    return GST_FLOW_OK;
}

void GStreamerWorker::on_gst_warning_message(GstBus* bus_, GstMessage* msg_, gpointer user_data_)
{
    Q_UNUSED(bus_);
    Q_UNUSED(user_data_);

    GError* err = nullptr;
    gchar* debug = nullptr;
    gst_message_parse_warning(msg_, &err, &debug);

    QString warning = err ? err->message : "Unknown Warning";
    if (warning.contains("timeout", Qt::CaseInsensitive) || warning.contains("connection", Qt::CaseInsensitive)
        || warning.contains("failed", Qt::CaseInsensitive))
    {
        RCLCPP_WARN(gst_logger, "GStreamer warning: %s", warning.toStdString().c_str());
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

static void on_decodebin_pad_added(GstElement* decodebin_, GstPad* pad_, gpointer user_data_)
{
    Q_UNUSED(decodebin_);
    auto* worker = static_cast<GStreamerWorker*>(user_data_);
    if (!worker)
    {
        RCLCPP_ERROR(gst_logger, "Invalid worker pointer in pad-added callback");
        return;
    }

    GstElement* queue0 = gst_bin_get_by_name(GST_BIN(worker->getPipeline()), "q0");
    if (!queue0)
    {
        RCLCPP_ERROR(gst_logger, "Failed to find q0 element");
        return;
    }

    GstPad* queueSinkPad = gst_element_get_static_pad(queue0, "sink");
    if (!queueSinkPad)
    {
        RCLCPP_ERROR(gst_logger, "Failed to get sink pad from q0");
        gst_object_unref(queue0);
        return;
    }

    if (gst_pad_link(pad_, queueSinkPad) != GST_PAD_LINK_OK)
    {
        RCLCPP_ERROR(gst_logger, "Failed to link decodebin pad to q0 sink pad");
    }
    else
    {
        RCLCPP_DEBUG(gst_logger, "Successfully linked decodebin pad");
    }

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
    Q_UNUSED(file_);
    Q_UNUSED(function_);
    Q_UNUSED(line_);
    Q_UNUSED(object_);
    Q_UNUSED(user_data_);

    if (level_ <= GST_LEVEL_ERROR)
    {
        const gchar* msg = gst_debug_message_get(message_);
        const gchar* cat = gst_debug_category_get_name(category_);

        QString msgStr(msg);
        if (!msgStr.contains("QoS", Qt::CaseInsensitive) && !msgStr.contains("latency", Qt::CaseInsensitive))
        {
            RCLCPP_ERROR(gst_logger, "[%s] %s", cat, msg);
        }
    }
}

GStreamerWorker::GStreamerWorker(QObject* parent):
    QObject(parent)
{
    qputenv("GST_DEBUG_NO_COLOR", "1");
    qputenv("GST_DEBUG", "1");

    gst_init(nullptr, nullptr);

    gst_debug_add_log_function(minimal_gst_debug_function, this, NULL);
    gst_debug_remove_log_function(gst_debug_log_default);

    // Only show errors
    gst_debug_set_default_threshold(GST_LEVEL_ERROR);

    // Mute specific noisy categories
    gst_debug_set_threshold_for_name("rtpjitterbuffer", GST_LEVEL_NONE);
    gst_debug_set_threshold_for_name("rtpsession", GST_LEVEL_NONE);
    gst_debug_set_threshold_for_name("rtpbasedepayload", GST_LEVEL_NONE);
    gst_debug_set_threshold_for_name("videodecoder", GST_LEVEL_NONE);
    gst_debug_set_threshold_for_name("basesink", GST_LEVEL_NONE);
    gst_debug_set_threshold_for_name("default", GST_LEVEL_NONE);

    RCLCPP_INFO(gst_logger, "GStreamer initialized with minimal logging");
}

GStreamerWorker::~GStreamerWorker()
{
    // Remove debug log function
    gst_debug_remove_log_function(minimal_gst_debug_function);

    // Then cleanup GStreamer
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
        RCLCPP_ERROR(gst_logger, "Invalid URL or missing host part: %s", rtspUrl_.toStdString().c_str());
        emit errorOccurred("Invalid URL format");
        return;
    }

    const QString pipelineDesc = this->buildPipelineString(rtspUrl_);
    _pipeline = gst_parse_launch(pipelineDesc.toUtf8().constData(), nullptr);

    if (!_pipeline)
    {
        RCLCPP_ERROR(gst_logger, "Failed to create pipeline");
        emit errorOccurred("Failed to create GStreamer pipeline");
        return;
    }

    GstElement* decodebin = gst_bin_get_by_name(GST_BIN(_pipeline), "dec");
    if (!decodebin)
    {
        RCLCPP_ERROR(gst_logger, "Failed to get decodebin element");
        emit errorOccurred("Failed to get decodebin element from pipeline");
        gst_object_unref(_pipeline);
        _pipeline = nullptr;
        return;
    }

    g_signal_connect(decodebin, "pad-added", G_CALLBACK(on_decodebin_pad_added), this);
    gst_object_unref(decodebin);

    GstElement* appSink = gst_bin_get_by_name(GST_BIN(_pipeline), "myappsink");
    if (!appSink)
    {
        RCLCPP_ERROR(gst_logger, "Failed to get appsink");
        emit errorOccurred("Failed to get appsink");
        gst_object_unref(_pipeline);
        _pipeline = nullptr;
        return;
    }

    g_object_set(G_OBJECT(appSink), "emit-signals", TRUE, "sync", FALSE, "max-buffers", 2, "drop", TRUE, nullptr);

    if (_newSampleSignalId != 0)
    {
        g_signal_handler_disconnect(appSink, _newSampleSignalId);
        _newSampleSignalId = 0;
    }

    _newSampleSignalId = g_signal_connect(appSink, "new-sample", G_CALLBACK(GStreamerWorker::on_new_sample), this);

    gst_object_unref(appSink);

    GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(_pipeline));
    if (!bus)
    {
        RCLCPP_ERROR(gst_logger, "Failed to get GStreamer bus");
        emit errorOccurred("Failed to get GStreamer bus");
        gst_object_unref(_pipeline);
        _pipeline = nullptr;
        return;
    }

    gst_bus_add_signal_watch(bus);

    _errorHandlerId = g_signal_connect(bus, "message::error", G_CALLBACK(GStreamerWorker::on_gst_error_message), this);
    _warningHandlerId = g_signal_connect(bus, "message::warning", G_CALLBACK(GStreamerWorker::on_gst_warning_message), this);

    g_object_unref(bus);

    _consecutiveErrorsCount = 0;

    // Set the pipeline to playing state
    GstStateChangeReturn ret = gst_element_set_state(_pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        RCLCPP_ERROR(gst_logger, "Failed to start pipeline");
        cleanupGStreamer();
        emit errorOccurred("Failed to start pipeline");
        return;
    }

    RCLCPP_INFO(gst_logger, "Pipeline started for URL: %s", rtspUrl_.toStdString().c_str());
    emit pipelineStarted(_pipeline);
}

void GStreamerWorker::stopPipeline()
{
    std::lock_guard<std::mutex> lock(_pipelineMutex);
    if (!_pipeline)
        return;

    gst_element_send_event(_pipeline, gst_event_new_eos());

    GstBus* bus = gst_element_get_bus(_pipeline);
    if (bus)
    {
        GstMessage* msg = gst_bus_timed_pop_filtered(bus, GST_SECOND, (GstMessageType)(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
        if (msg)
            gst_message_unref(msg);
        gst_object_unref(bus);
    }

    RCLCPP_INFO(gst_logger, "Stopping pipeline");
    this->cleanupGStreamer();
    emit pipelineStopped();
}

void GStreamerWorker::cleanupGStreamer()
{
    GstElement* pipeline_to_clean = nullptr;
    
    {
        std::lock_guard<std::mutex> lock(_pipelineMutex);
        if (!_pipeline)
            return;
        pipeline_to_clean = _pipeline;
        _pipeline = nullptr; 
    }

    if (pipeline_to_clean)
    {
        GstElement* rtspsrc = gst_bin_get_by_name(GST_BIN(pipeline_to_clean), "rtspsrc0");
        if (rtspsrc)
        {
            gst_element_send_event(rtspsrc, gst_event_new_eos());
            gst_object_unref(rtspsrc);
        }

        GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline_to_clean));
        if (bus)
        {
            if (_errorHandlerId)
            {
                g_signal_handler_disconnect(bus, _errorHandlerId);
                _errorHandlerId = 0;
            }
            if (_warningHandlerId)
            {
                g_signal_handler_disconnect(bus, _warningHandlerId);
                _warningHandlerId = 0;
            }
            gst_bus_remove_signal_watch(bus);
            gst_object_unref(bus);
        }

        GstElement* appSink = gst_bin_get_by_name(GST_BIN(pipeline_to_clean), "myappsink");
        if (appSink && _newSampleSignalId != 0)
        {
            g_signal_handler_disconnect(appSink, _newSampleSignalId);
            _newSampleSignalId = 0;
            gst_object_unref(appSink);
        }

        GstStateChangeReturn ret = gst_element_set_state(pipeline_to_clean, GST_STATE_NULL);
        if (ret == GST_STATE_CHANGE_ASYNC)
        {
            GstState state, pending;
            ret = gst_element_get_state(pipeline_to_clean, &state, &pending, 5 * GST_SECOND);
            if (ret == GST_STATE_CHANGE_FAILURE)
            {
                RCLCPP_ERROR(gst_logger, "Failed to stop pipeline cleanly");
            }
        }

        gst_object_unref(pipeline_to_clean);
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