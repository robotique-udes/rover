#include "QGStreamerWorker.hpp"
#include "QLoggingMacros.hpp"
#include <QDebug>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/videooverlay.h>

static void on_gst_error_message(GstBus* bus, GstMessage* msg, gpointer user_data)
{
    Q_UNUSED(bus);
    auto* worker = static_cast<GStreamerWorker*>(user_data);

    GError* err = nullptr;
    gchar* debug = nullptr;
    gst_message_parse_error(msg, &err, &debug);

    QString errorMsg = QString("%1").arg(err ? err->message : "Unknown Error");

    LOG_DEBUG("GStreamer", errorMsg);

    if (worker)
    {
        emit worker->errorOccurred(errorMsg);
    }

    if (err)
        g_error_free(err);
    if (debug)
        g_free(debug);
}

static GstFlowReturn on_new_sample(GstElement* sink, gpointer user_data)
{
    auto* worker = static_cast<GStreamerWorker*>(user_data);
    if (!worker)
    {
        return GST_FLOW_OK;
    }

    emit worker->frameReceived();

    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (sample)
    {
        gst_sample_unref(sample);
    }

    return GST_FLOW_OK;
}

static void on_gst_warning_message(GstBus* bus, GstMessage* msg)
{
    Q_UNUSED(bus);
    GError* err = nullptr;
    gchar* debug = nullptr;
    gst_message_parse_warning(msg, &err, &debug);

    LOG_DEBUG("GStreamer", QString("%1").arg(err ? err->message : "Unknown Warning"));

    if (err)
        g_error_free(err);
    if (debug)
        g_free(debug);
}

static void on_decodebin_pad_added(GstElement* decodebin, GstPad* pad, gpointer user_data)
{
    Q_UNUSED(decodebin);
    auto* worker = static_cast<GStreamerWorker*>(user_data);
    if (!worker)
    {
        LOG_ERROR("GStreamer", "Invalid worker pointer in pad-added callback");
        return;
    }

    GstElement* queue0 = gst_bin_get_by_name(GST_BIN(worker->getPipeline()), "q0");
    if (!queue0)
    {
        LOG_ERROR("GStreamer", "Failed to find q0 element");
        return;
    }

    GstPad* queueSinkPad = gst_element_get_static_pad(queue0, "sink");
    if (!queueSinkPad)
    {
        LOG_ERROR("GStreamer", "Failed to get sink pad from q0");
        gst_object_unref(queue0);
        return;
    }

    if (gst_pad_link(pad, queueSinkPad) != GST_PAD_LINK_OK)
    {
        LOG_ERROR("GStreamer", "Failed to link decodebin pad to q0 sink pad");
    }

    gst_object_unref(queueSinkPad);
    gst_object_unref(queue0);
}

GStreamerWorker::GStreamerWorker(QObject* parent): QObject(parent)
{
    gst_init(nullptr, nullptr);
}

GStreamerWorker::~GStreamerWorker()
{
    cleanupGStreamer();
}

QString GStreamerWorker::buildPipelineString(const QString& rtspUrl) const
{
    return QString("rtspsrc location=%1 latency=50 timeout=5000000 buffer-mode=none do-retransmission=false drop-on-latency=true ! decodebin "
                   "name=dec "
                   "queue name=q0 max-size-buffers=10 max-size-time=0 max-size-bytes=0 leaky=downstream ! videoconvert ! tee name=t "
                   "t. ! queue max-size-buffers=2 leaky=downstream ! ximagesink sync=false "
                   "t. ! queue max-size-buffers=2 leaky=downstream ! videoconvert ! appsink name=myappsink sync=false")
        .arg(rtspUrl);
}

//QString GStreamerWorker::buildPipelineString(const QString& rtspUrl) const
//{
    //return QString("rtspsrc location=%1 latency=50 timeout=5000000 "
                  //"buffer-mode=none do-retransmission=false drop-on-latency=true "
                  //"! rtph264depay ! h264parse ! "
                  //"decodebin name=dec max-size-buffers=1 "
                  //"! queue name=q0 max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream "
                  //"! videoconvert max-threads=1 ! video/x-raw,format=I420 ! tee name=t "
                  //"t. ! queue max-size-buffers=1 leaky=downstream ! videoconvert ! "
                  //"ximagesink sync=false name=videosink " // Use ximagesink instead of xvimagesink
                  //"t. ! queue max-size-buffers=1 leaky=downstream ! videoconvert ! "
                  //"appsink name=myappsink sync=false max-buffers=1 drop=true")
        //.arg(rtspUrl);
//}

void GStreamerWorker::startPipeline(const QString& rtspUrl)
{
    cleanupGStreamer();

    const QString pipelineDesc = buildPipelineString(rtspUrl);
    m_pipeline = gst_parse_launch(pipelineDesc.toUtf8().constData(), nullptr);

    if (!m_pipeline)
    {
        LOG_ERROR("GStreamer", "Failed to create pipeline");
        emit errorOccurred("Failed to create GStreamer pipeline");
        return;
    }

    GstElement* decodebin = gst_bin_get_by_name(GST_BIN(m_pipeline), "dec");
    if (!decodebin)
    {
        LOG_ERROR("GStreamer", "Failed to get decodebin element");
        emit errorOccurred("Failed to get decodebin element from pipeline");
        return;
    }

    g_signal_connect(decodebin, "pad-added", G_CALLBACK(on_decodebin_pad_added), this);
    gst_object_unref(decodebin);

    GstElement* appSink = gst_bin_get_by_name(GST_BIN(m_pipeline), "myappsink");
    if (!appSink)
    {
        LOG_ERROR("GStreamer", "Failed to get appsink");
        emit errorOccurred("Failed to get appsink");
        return;
    }
    
    // Configure once with all settings
    g_object_set(G_OBJECT(appSink), 
        "emit-signals", TRUE, 
        "sync", FALSE,
        "max-buffers", 2,  
        "drop", TRUE,      
        nullptr);
    
    static gulong signal_id = 0;
    if (signal_id != 0) {
        g_signal_handler_disconnect(appSink, signal_id);
    }

    if (newSampleSignalId != 0) {
        g_signal_handler_disconnect(appSink, newSampleSignalId);
        newSampleSignalId = 0;
    }
    
    newSampleSignalId = g_signal_connect(appSink, "new-sample", G_CALLBACK(on_new_sample), this);
    
    
    // Unref exactly once
    gst_object_unref(appSink);
    

    //g_object_set(G_OBJECT(appSink), "emit-signals", TRUE, "sync", FALSE, nullptr);
    //g_signal_connect(appSink, "new-sample", G_CALLBACK(on_new_sample), this);
    //gst_object_unref(appSink);

    GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(m_pipeline));
    if (!bus)
    {
        LOG_ERROR("GStreamer", "Failed to get GStreamer bus");
        emit errorOccurred("Failed to get GStreamer bus");
        return;
    }

    gst_bus_add_signal_watch(bus);

    g_signal_connect(bus, "message::error", G_CALLBACK(on_gst_error_message), this);
    g_signal_connect(bus, "message::warning", G_CALLBACK(on_gst_warning_message), this);

    g_object_unref(bus);

    LOG_DEBUG("GstreamerWorker", "Pipeline started");
    emit pipelineStarted(m_pipeline);
}

void GStreamerWorker::stopPipeline()
{
    cleanupGStreamer();
    emit pipelineStopped();
}

void GStreamerWorker::cleanupGStreamer()
{
    if (m_pipeline)
    {
        // First set pipeline to NULL state
        GstStateChangeReturn ret = gst_element_set_state(m_pipeline, GST_STATE_NULL);
        
        // Wait for state change to complete
        if (ret == GST_STATE_CHANGE_ASYNC) {
            gst_element_get_state(m_pipeline, NULL, NULL, GST_CLOCK_TIME_NONE);
        }
        
        // Get the bus and remove watch
        GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(m_pipeline));
        if (bus) {
            gst_bus_remove_signal_watch(bus);
            gst_object_unref(bus);
        }
        
        // Unreference the pipeline
        gst_object_unref(m_pipeline);
        m_pipeline = nullptr;
        
    }
}