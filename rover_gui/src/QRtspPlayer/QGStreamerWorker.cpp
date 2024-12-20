#include "QGStreamerWorker.hpp"
#include <gst/gst.h>
#include <gst/video/videooverlay.h>
#include <QDebug>

static void on_gst_error_message(GstBus* bus, GstMessage* msg, gpointer user_data) {
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


static void on_gst_eos_message(GstBus* bus, GstMessage* msg, gpointer user_data) {
 
    GStreamerWorker* worker = static_cast<GStreamerWorker*>(user_data);
    if (worker) {
        // Could emit a signal if EOS needs handling.
        // emit worker->pipelineStopped(); // If desired
    }
}

static void on_gst_warning_message(GstBus* bus, GstMessage* msg, gpointer user_data) {
    GError* err = nullptr;
    gchar* debug = nullptr;
    gst_message_parse_warning(msg, &err, &debug);

    qDebug() << "GStreamer warning message callback triggered.";
    qDebug() << "Warning:" << (err ? err->message : "Unknown") << "Debug:" << (debug ? debug : "None");

    if (err) g_error_free(err);
    if (debug) g_free(debug);
}

static void on_gst_state_changed_message(GstBus* bus, GstMessage* msg, gpointer user_data) {
    if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_STATE_CHANGED) {
        GstState old_state, new_state, pending_state;
        gst_message_parse_state_changed(msg, &old_state, &new_state, &pending_state);
        qDebug() << "GStreamer state changed from" << gst_element_state_get_name(old_state)
                 << "to" << gst_element_state_get_name(new_state) << "pending" << gst_element_state_get_name(pending_state);

        // This might help you see if the pipeline moves into a different state instead of erroring.
    }
}

static void on_decodebin_pad_added(GstElement *decodebin, GstPad *pad, gpointer user_data) {
    Q_UNUSED(decodebin);

    GStreamerWorker* worker = static_cast<GStreamerWorker*>(user_data);
    if (!worker) return;

    // At this point, a new pad means a stream has been found.
    qDebug() << "Decodebin pad-added: Stream found!";

    // Emit a signal to notify that we have a real stream now.
    emit worker->streamFound();
}


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

    pipeline = gst_parse_launch(
        QString("rtspsrc location=%1 latency=50 ! decodebin name=dec ! videoconvert ! xvimagesink sync=false")
        .arg(rtspUrl)
        .toUtf8().constData(),
        nullptr
    );

    GstElement* decodebin = gst_bin_get_by_name(GST_BIN(pipeline), "dec");
    if (!decodebin) {
        emit errorOccurred("Failed to get decodebin element from pipeline.");
        return;
    }

    // Connect to the pad-added signal of decodebin
    g_signal_connect(decodebin, "pad-added", G_CALLBACK(on_decodebin_pad_added), this);
    gst_object_unref(decodebin);

    qDebug() << "Pipeline created:" << (pipeline != nullptr);

    if (!pipeline) {
        emit errorOccurred("Failed to create GStreamer pipeline.");
        return;
    }

    GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    if (!bus) {
        qDebug() << "Failed to get GStreamer bus.";
        emit errorOccurred("Failed to get GStreamer bus.");
        return;
    }

    qDebug() << "Bus retrieved successfully";

    // `gst_bus_add_signal_watch()` returns void, just call it
    gst_bus_add_signal_watch(bus);
    qDebug() << "Signal watch added";

    g_signal_connect(bus, "message::error", G_CALLBACK(on_gst_error_message), this);
    g_signal_connect(bus, "message::eos", G_CALLBACK(on_gst_eos_message), this);
    g_signal_connect(bus, "message::warning", G_CALLBACK(on_gst_warning_message), this);
    g_signal_connect(bus, "message::state-changed", G_CALLBACK(on_gst_state_changed_message), this);

    g_object_unref(bus);

    emit pipelineStarted(pipeline);
}


void GStreamerWorker::stopPipeline()
{
    cleanupGStreamer();
    emit pipelineStopped();
}
