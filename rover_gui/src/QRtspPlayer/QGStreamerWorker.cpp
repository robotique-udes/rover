#include "QGStreamerWorker.hpp"
#include <gst/gst.h>
#include <gst/video/videooverlay.h>
#include <QDebug>
#include <gst/app/gstappsink.h>

// Forward declare callbacks
static void on_gst_error_message(GstBus* bus, GstMessage* msg, gpointer user_data);
static GstFlowReturn on_new_sample(GstElement* sink, gpointer user_data);
static void on_gst_eos_message(GstBus* bus, GstMessage* msg, gpointer user_data);
static void on_gst_warning_message(GstBus* bus, GstMessage* msg, gpointer user_data);
static void on_gst_state_changed_message(GstBus* bus, GstMessage* msg, gpointer user_data);
static void on_decodebin_pad_added(GstElement *decodebin, GstPad *pad, gpointer user_data);

GStreamerWorker::GStreamerWorker(QObject* parent)
    : QObject(parent), pipeline(nullptr)
{
    gst_init(nullptr, nullptr);
}

GStreamerWorker::~GStreamerWorker()
{
    cleanupGStreamer();
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

    // Build the pipeline string without directly linking decodebin's pad
    pipeline = gst_parse_launch(
        QString(
            "rtspsrc location=%1 latency=50 timeout=5000000 ! decodebin name=dec "
            "queue name=q0 ! videoconvert ! tee name=t "
            "t. ! queue ! xvimagesink sync=false "
            "t. ! queue ! videoconvert ! appsink name=myappsink sync=false"
        )
        .arg(rtspUrl)
        .toUtf8().constData(),
        nullptr
    );

    qDebug() << "Pipeline created:" << (pipeline != nullptr);
    if (!pipeline) {
        emit errorOccurred("Failed to create GStreamer pipeline.");
        return;
    }

    // Get decodebin
    GstElement* decodebin = gst_bin_get_by_name(GST_BIN(pipeline), "dec");
    if (!decodebin) {
        emit errorOccurred("Failed to get decodebin element from pipeline.");
        return;
    }

    // Connect pad-added signal
    g_signal_connect(decodebin, "pad-added", G_CALLBACK(on_decodebin_pad_added), this);
    gst_object_unref(decodebin);

    // Get the appsink element
    GstElement* appSink = gst_bin_get_by_name(GST_BIN(pipeline), "myappsink");
    if (!appSink) {
        emit errorOccurred("Failed to get appsink");
        return;
    }

    // Set appsink properties
    g_object_set(G_OBJECT(appSink),
                 "emit-signals", TRUE,
                 "sync", FALSE,
                 nullptr);

    g_signal_connect(appSink, "new-sample", G_CALLBACK(on_new_sample), this);
    gst_object_unref(appSink);

    // Set up the bus and signal watches
    GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    if (!bus) {
        qDebug() << "Failed to get GStreamer bus.";
        emit errorOccurred("Failed to get GStreamer bus.");
        return;
    }

    qDebug() << "Bus retrieved successfully";
    gst_bus_add_signal_watch(bus);
    qDebug() << "Signal watch added";

    g_signal_connect(bus, "message::error", G_CALLBACK(on_gst_error_message), this);
    g_signal_connect(bus, "message::eos", G_CALLBACK(on_gst_eos_message), this);
    g_signal_connect(bus, "message::warning", G_CALLBACK(on_gst_warning_message), this);
    g_signal_connect(bus, "message::state-changed", G_CALLBACK(on_gst_state_changed_message), this);

    g_object_unref(bus);

    // Now that pipeline and bus are ready, emit pipelineStarted
    emit pipelineStarted(pipeline);
}

void GStreamerWorker::stopPipeline()
{
    cleanupGStreamer();
    emit pipelineStopped();
}

// Callbacks
static void on_gst_error_message(GstBus* bus, GstMessage* msg, gpointer user_data) {
    Q_UNUSED(bus);
    qDebug() << "GStreamer error message callback triggered.";
    GError* err = nullptr;
    gchar* debug = nullptr;
    gst_message_parse_error(msg, &err, &debug);

    GStreamerWorker* worker = static_cast<GStreamerWorker*>(user_data);
    if (worker) {
        QString errorString = QString("GStreamer error: %1").arg(err->message);
        qDebug() << "Emitting errorOccurred with message:" << errorString;
        emit worker->errorOccurred(errorString);
    }

    if (err) g_error_free(err);
    if (debug) g_free(debug);
}

static GstFlowReturn on_new_sample(GstElement* sink, gpointer user_data) {
    GStreamerWorker* worker = static_cast<GStreamerWorker*>(user_data);
    if (!worker) return GST_FLOW_OK;

    emit worker->frameReceived();
    
    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (sample) {
        gst_sample_unref(sample);
    }

    return GST_FLOW_OK;
}

static void on_gst_eos_message(GstBus* bus, GstMessage* msg, gpointer user_data) {
    Q_UNUSED(bus);
    Q_UNUSED(msg);
    GStreamerWorker* worker = static_cast<GStreamerWorker*>(user_data);
    if (worker) {
        // End-of-stream logic if needed
    }
}

static void on_gst_warning_message(GstBus* bus, GstMessage* msg, gpointer user_data) {
    Q_UNUSED(bus);
    Q_UNUSED(user_data);
    GError* err = nullptr;
    gchar* debug = nullptr;
    gst_message_parse_warning(msg, &err, &debug);

    qDebug() << "GStreamer warning message callback triggered.";
    qDebug() << "Warning:" << (err ? err->message : "Unknown") << "Debug:" << (debug ? debug : "None");

    if (err) g_error_free(err);
    if (debug) g_free(debug);
}

static void on_gst_state_changed_message(GstBus* bus, GstMessage* msg, gpointer user_data) {
    Q_UNUSED(bus);
    Q_UNUSED(user_data);
    if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_STATE_CHANGED) {
        GstState old_state, new_state, pending_state;
        gst_message_parse_state_changed(msg, &old_state, &new_state, &pending_state);
        qDebug() << "GStreamer state changed from" << gst_element_state_get_name(old_state)
                 << "to" << gst_element_state_get_name(new_state) << "pending" << gst_element_state_get_name(pending_state);
    }
}

static void on_decodebin_pad_added(GstElement *decodebin, GstPad *pad, gpointer user_data) {
    Q_UNUSED(decodebin);
    GStreamerWorker* worker = static_cast<GStreamerWorker*>(user_data);
    if (!worker) return;

    qDebug() << "Decodebin pad-added: Stream found!";
    emit worker->streamFound();

    // Link decodebin's new pad to q0 sink pad
    GstElement* queue0 = gst_bin_get_by_name(GST_BIN(worker->pipeline), "q0");
    if (!queue0) {
        g_warning("Failed to find q0 element");
        return;
    }

    GstPad* queueSinkPad = gst_element_get_static_pad(queue0, "sink");
    if (!queueSinkPad) {
        g_warning("Failed to get sink pad from q0");
        gst_object_unref(queue0);
        return;
    }

    if (gst_pad_link(pad, queueSinkPad) != GST_PAD_LINK_OK) {
        g_warning("Failed to link decodebin pad to q0 sink pad.");
    } else {
        qDebug() << "Successfully linked decodebin pad to q0.";
    }

    gst_object_unref(queueSinkPad);
    gst_object_unref(queue0);
}
