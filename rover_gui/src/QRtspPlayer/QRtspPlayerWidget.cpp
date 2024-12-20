#include "QRtspPlayerWidget.hpp"
#include <gst/video/videooverlay.h>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>

RtspPlayerWidget::RtspPlayerWidget(QWidget* parent)
    : QWidget(parent), pipeline(nullptr)
{

    ui = new Ui::RtspPlayerWidget(); 
    ui->setupUi(this);

    // Create worker thread and worker object
    workerThread = new QThread(this);
    gstreamerWorker = new GStreamerWorker();
    gstreamerWorker->moveToThread(workerThread);

    connect(workerThread, &QThread::finished, gstreamerWorker, &QObject::deleteLater);
    connect(this, QOverload<const QString&>::of(&RtspPlayerWidget::requestStartStream),
        gstreamerWorker, &GStreamerWorker::startPipeline);
    connect(this, &RtspPlayerWidget::requestStopStream, gstreamerWorker, &GStreamerWorker::stopPipeline);
    connect(gstreamerWorker, &GStreamerWorker::pipelineStarted, this, &RtspPlayerWidget::onPipelineStarted);
    connect(gstreamerWorker, &GStreamerWorker::errorOccurred, this, &RtspPlayerWidget::onErrorOccurred);
    connect(ui->startButton, &QPushButton::clicked, this, [this]() {
    // When start is clicked, call startStream with the entered URL
    startStream(ui->rtspUrlInput->text());
    });

    connect(ui->stopButton, &QPushButton::clicked, this, &RtspPlayerWidget::stopStream);
    workerThread->start();
    
}

RtspPlayerWidget::~RtspPlayerWidget()
{
    workerThread->quit();
    workerThread->wait();
    delete ui;
}


void RtspPlayerWidget::initializeGStreamer()
{
    gst_init(nullptr, nullptr);
}

void RtspPlayerWidget::cleanupGStreamer()
{
    if (pipeline) {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
        pipeline = nullptr;
    }
}

void RtspPlayerWidget::startStream(const QString& rtspUrl)
{
    if (rtspUrl.isEmpty()) {
        QMessageBox::warning(this, "Error", "RTSP URL cannot be empty.");
        return;
    }

    emit requestStartStream(rtspUrl); 
}

void RtspPlayerWidget::stopStream()
{
    emit requestStopStream();
}

void RtspPlayerWidget::onPipelineStarted(GstElement* receivedPipeline)
{
    if (!receivedPipeline) {
        QMessageBox::critical(this, "Error", "No pipeline received.");
        return;
    }
    this->pipeline = receivedPipeline; // Store the pipeline locally

    // Retrieve the video sink implementing GST_TYPE_VIDEO_OVERLAY
    GstElement* videoSink = gst_bin_get_by_interface(GST_BIN(pipeline), GST_TYPE_VIDEO_OVERLAY);
    if (!videoSink) {
        QMessageBox::critical(this, "Error", "Failed to configure video sink.");
        return;
    }

    // Ensure our widget has a native window ID
    ui->videoWidget->winId();

    // Assign the window handle to the video sink
    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(videoSink),
        (guintptr)ui->videoWidget->winId());

    // Now start playing
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    QMessageBox::information(this, "Pipeline", "Pipeline started successfully!");
}

void RtspPlayerWidget::onErrorOccurred(const QString& error)
{
    QMessageBox::critical(this, "Error", error);
}