#include "QRtspPlayerWidget.hpp"
#include <gst/video/videooverlay.h>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QStyle>
#include <QApplication>
#include <QHBoxLayout>
#include <QDebug>
#include <QTimer>

RtspPlayerWidget::RtspPlayerWidget(QWidget* parent)
    : QWidget(parent), pipeline(nullptr), receivingFrames(false)
{
    ui = new Ui::RtspPlayerWidget(); 
    ui->setupUi(this);

    // Worker thread and object
    workerThread = new QThread(this);
    gstreamerWorker = new GStreamerWorker();
    gstreamerWorker->moveToThread(workerThread);

    connect(workerThread, &QThread::finished, gstreamerWorker, &QObject::deleteLater);

    // Start/Stop and error signals
    connect(this, QOverload<const QString&>::of(&RtspPlayerWidget::requestStartStream),
            gstreamerWorker, &GStreamerWorker::startPipeline);
    connect(this, &RtspPlayerWidget::requestStopStream,
            gstreamerWorker, &GStreamerWorker::stopPipeline);
    connect(gstreamerWorker, &GStreamerWorker::pipelineStarted,
            this, &RtspPlayerWidget::onPipelineStarted);
    connect(gstreamerWorker, &GStreamerWorker::errorOccurred,
            this, &RtspPlayerWidget::onErrorOccurred);

    // Stream found means decodebin found a stream, stay yellow until frames arrive
    connect(gstreamerWorker, &GStreamerWorker::streamFound, this, [this]() {
        qDebug() << "Stream found, waiting for frames...";
        // Don't turn green yet, need actual frames
    });

    // Timer to detect frame loss (no frames for some time)
    frameTimeoutTimer = new QTimer(this);
    frameTimeoutTimer->setSingleShot(true);
    connect(frameTimeoutTimer, &QTimer::timeout, this, [this]() {
        // No frames for timeout → turn red
        
        ui->statusIndicator->setStyleSheet("QFrame { border-radius: 10px; background-color: red; }");
        receivingFrames = false;
    });

    // frameReceived: Actual frames coming in
    connect(gstreamerWorker, &GStreamerWorker::frameReceived, this, [this]() {
        receivingFrames = true;
        // Since we have frames, reset the timer to detect future stoppages
        frameTimeoutTimer->start(2000); // If no frames for 2s → red

        // If currently yellow, now we can confidently go green
        if (ui->statusIndicator->styleSheet().contains("yellow") ||
            ui->statusIndicator->styleSheet().contains("red")) {
            ui->statusIndicator->setStyleSheet("QFrame { border-radius: 10px; background-color: green; }");
        }
    });

    // UI button signals
    connect(ui->startButton, &QPushButton::clicked, this, [this]() {
        startStream(ui->rtspUrlInput->text());
    });

    connect(ui->stopButton, &QPushButton::clicked, this, &RtspPlayerWidget::stopStream);

    // Reconnect timer for after errors
    reconnectTimer = new QTimer(this);
    reconnectTimer->setSingleShot(true);
    connect(reconnectTimer, &QTimer::timeout, this, [this]() {
        // If no frames reappeared, turn red
        if (!receivingFrames) {
            ui->statusIndicator->setStyleSheet("QFrame { border-radius: 10px; background-color: red; }");
        }
    });

    workerThread->start();
}

RtspPlayerWidget::~RtspPlayerWidget()
{
    workerThread->quit();
    workerThread->wait();
    delete ui;
}

void RtspPlayerWidget::startStream(const QString& rtspUrl)
{
    if (rtspUrl.isEmpty()) {
        QMessageBox::warning(this, "Error", "RTSP URL cannot be empty.");
        return;
    }

    // Starting to connect: yellow
    ui->statusIndicator->setStyleSheet("QFrame { border-radius: 10px; background-color: yellow; }");
    receivingFrames = false;
    emit requestStartStream(rtspUrl); 
}

void RtspPlayerWidget::stopStream()
{
    emit requestStopStream();
    // Stopping the stream: no frames expected
    // Set yellow indicating not streaming
    ui->statusIndicator->setStyleSheet("QFrame { border-radius: 10px; background-color: yellow; }");
    receivingFrames = false;
    // Stop frame timer since we deliberately stopped
    frameTimeoutTimer->stop();
}

void RtspPlayerWidget::onPipelineStarted(GstElement* receivedPipeline)
{
    if (!receivedPipeline) {
        QMessageBox::critical(this, "Error", "No pipeline received.");
        return;
    }

    this->pipeline = receivedPipeline; 

    GstElement* videoSink = gst_bin_get_by_interface(GST_BIN(pipeline), GST_TYPE_VIDEO_OVERLAY);
    if (!videoSink) {
        QMessageBox::critical(this, "Error", "Failed to configure video sink.");
        return;
    }

    ui->videoWidget->winId();
    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(videoSink),
        (guintptr)ui->videoWidget->winId());

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    // Pipeline started, but no frames yet. Keep yellow.
    // Start a longer timeout to see if frames come in.
    receivingFrames = false;
    frameTimeoutTimer->start(5000); // If no frames after 5s → red
}

void RtspPlayerWidget::onErrorOccurred(const QString& error)
{
    qDebug() << "onErrorOccurred called with error:" << error;
    // Error means stream is lost. Turn yellow (attempting recovery or waiting)
    ui->statusIndicator->setStyleSheet("QFrame { border-radius: 10px; background-color: yellow; }");
    receivingFrames = false;

    // Start a reconnect timer. If no frames arrive before it expires, go red
    reconnectTimer->start(5000);

    QMessageBox::critical(this, "Error", error);
}
