#include "QRtspPlayerWidget.hpp"
#include "QLoggingMacros.hpp"
#include <QMessageBox>
#include <gst/video/videooverlay.h>

RtspPlayerWidget::RtspPlayerWidget(QWidget* parent):
    QWidget(parent),
    ui(new Ui::RtspPlayerWidget),
    workerThread(new QThread(this)),
    gstreamerWorker(new GStreamerWorker()),
    reconnectTimer(new QTimer(this)),
    frameTimeoutTimer(new QTimer(this)),
    pipeline(nullptr),
    receivingFrames(false)
{
    ui->setupUi(this);

    gstreamerWorker->moveToThread(workerThread);
    connect(workerThread, &QThread::finished, gstreamerWorker, &QObject::deleteLater);

    connect(this, &RtspPlayerWidget::requestStartStream, gstreamerWorker, &GStreamerWorker::startPipeline);
    connect(this, &RtspPlayerWidget::requestStopStream, gstreamerWorker, &GStreamerWorker::stopPipeline);

    connect(gstreamerWorker, &GStreamerWorker::pipelineStarted, this, &RtspPlayerWidget::onPipelineStarted);
    connect(gstreamerWorker, &GStreamerWorker::errorOccurred, this, &RtspPlayerWidget::onErrorOccurred);

    connect(gstreamerWorker,
            &GStreamerWorker::frameReceived,
            this,
            [this]()
            {
                if (!receivingFrames)
                {
                    LOG_INFO("RtspPlayer", "Receiving frames..");
                    receivingFrames = true;
                }
                frameTimeoutTimer->start(2000);
                updateStatusIndicator("green");
            });

    frameTimeoutTimer->setSingleShot(true);
    connect(frameTimeoutTimer,
            &QTimer::timeout,
            this,
            [this]()
            {
                if (receivingFrames)
                {
                    receivingFrames = false;
                    LOG_WARNING("RtspPlayer", "Frame timeout - no frames received");
                    updateStatusIndicator("red");
                }
            });

    reconnectTimer->setSingleShot(true);
    connect(reconnectTimer,
            &QTimer::timeout,
            this,
            [this]()
            {
                if (!receivingFrames)
                {
                    LOG_WARNING("RtspPlayer", "Stream reconnection failed");
                    updateStatusIndicator("red");
                }
            });

    connect(ui->startButton, &QPushButton::clicked, this, [this]() { startStream(ui->rtspUrlInput->text()); });

    connect(ui->stopButton, &QPushButton::clicked, this, &RtspPlayerWidget::stopStream);

    workerThread->start();
    updateStatusIndicator("yellow");
}

RtspPlayerWidget::~RtspPlayerWidget()
{
    stopStream();
    workerThread->quit();
    workerThread->wait();
    delete ui;
}

void RtspPlayerWidget::startStream(const QString& rtspUrl)
{
    if (rtspUrl.isEmpty())
    {
        LOG_WARNING("RtspPlayer", "Empty RTSP URL provided");
        QMessageBox::warning(this, "Error", "RTSP URL cannot be empty.");
        return;
    }

    LOG_INFO("RtspPlayer", QString("Starting stream: %1").arg(rtspUrl));
    receivingFrames = false;
    updateStatusIndicator("yellow");
    emit requestStartStream(rtspUrl);
}

void RtspPlayerWidget::stopStream()
{
    if (!pipeline && !receivingFrames)
    {
        return;
    }

    LOG_INFO("RtspPlayer", "Stopping stream");
    emit requestStopStream();
    receivingFrames = false;

    frameTimeoutTimer->stop();
    updateStatusIndicator("yellow");
}

void RtspPlayerWidget::onPipelineStarted(GstElement* receivedPipeline)
{
    if (!receivedPipeline)
    {
        LOG_ERROR("RtspPlayer", "Pipeline creation failed");
        QMessageBox::critical(this, "Error", "No pipeline received from worker.");
        updateStatusIndicator("red");
        return;
    }

    pipeline = receivedPipeline;

    GstElement* videoSink = gst_bin_get_by_interface(GST_BIN(pipeline), GST_TYPE_VIDEO_OVERLAY);
    if (!videoSink)
    {
        LOG_ERROR("RtspPlayer", "Failed to find VideoOverlay in pipeline");
        QMessageBox::critical(this, "Error", "Failed to find a VideoOverlay in the pipeline.");
        updateStatusIndicator("red");
        return;
    }

    ui->videoWidget->winId();
    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(videoSink), (guintptr)ui->videoWidget->winId());
    gst_object_unref(videoSink);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    LOG_DEBUG("RtspPlayer", "Pipeline state set to PLAYING");

    receivingFrames = false;
    frameTimeoutTimer->start(5000);
}

void RtspPlayerWidget::onErrorOccurred(const QString& error)
{
    if (!receivingFrames)
    {
        LOG_ERROR("RtspPlayer", error);
        updateStatusIndicator("yellow");
        reconnectTimer->start(5000);
        QMessageBox::critical(this, "Error", error);
    }
}

void RtspPlayerWidget::updateStatusIndicator(const QString& color)
{
    ui->statusIndicator->setStyleSheet(QString("QFrame { border-radius: 10px; background-color: %1; }").arg(color));
}