#include "QRtspPlayerWidget.hpp"
#include <gst/video/videooverlay.h>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QStyle>
#include <QApplication>
#include <QHBoxLayout>
#include <QDebug>

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

    // Start/Stop and error signals
    connect(this, QOverload<const QString&>::of(&RtspPlayerWidget::requestStartStream),
            gstreamerWorker, &GStreamerWorker::startPipeline);
    connect(this, &RtspPlayerWidget::requestStopStream,
            gstreamerWorker, &GStreamerWorker::stopPipeline);
    connect(gstreamerWorker, &GStreamerWorker::pipelineStarted,
            this, &RtspPlayerWidget::onPipelineStarted);
    connect(gstreamerWorker, &GStreamerWorker::errorOccurred,
            this, &RtspPlayerWidget::onErrorOccurred);
    connect(gstreamerWorker, &GStreamerWorker::streamFound, this, [this]() {
    // Only set green if no error has occurred
    ui->statusIndicator->setStyleSheet("QFrame { border-radius: 10px; background-color: green; }");
    });

    // UI button signals
    connect(ui->startButton, &QPushButton::clicked, this, [this]() {
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

void RtspPlayerWidget::startStream(const QString& rtspUrl)
{
    if (rtspUrl.isEmpty()) {
        QMessageBox::warning(this, "Error", "RTSP URL cannot be empty.");
        return;
    }

    ui->statusIndicator->setStyleSheet("QFrame { border-radius: 10px; background-color: yellow; }");
    emit requestStartStream(rtspUrl); 
}

void RtspPlayerWidget::stopStream()
{
    emit requestStopStream();
    ui->statusIndicator->setStyleSheet("QFrame { border-radius: 10px; background-color: red; }");
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
    
}

void RtspPlayerWidget::onErrorOccurred(const QString& error)
{
    qDebug() << "onErrorOccurred called with error:" << error;
    ui->statusIndicator->setStyleSheet("QFrame { border-radius: 10px; background-color: red; }");
    QMessageBox::critical(this, "Error", error);
}

