#include "QRtspPlayer.hpp"
#include <gst/video/videooverlay.h>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>

RtspPlayerWidget::RtspPlayerWidget(QWidget* parent)
    : QWidget(parent), pipeline(nullptr)
{
    // UI Elements
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    QHBoxLayout* topLayout = new QHBoxLayout();

    rtspUrlInput = new QLineEdit(this);
    rtspUrlInput->setPlaceholderText("Enter RTSP URL...");
    topLayout->addWidget(rtspUrlInput);

    startButton = new QPushButton(this);
    startButton->setIcon(QIcon(":/play.png"));
    topLayout->addWidget(startButton);

    stopButton = new QPushButton(this);
    stopButton->setIcon(QIcon(":/stop.png"));
    topLayout->addWidget(stopButton);

    mainLayout->addLayout(topLayout);

    videoWidget = new QWidget(this);
    videoWidget->setStyleSheet("background-color: black;");
    mainLayout->addWidget(videoWidget);

    connect(startButton, &QPushButton::clicked, this, &RtspPlayerWidget::startStream);
    connect(stopButton, &QPushButton::clicked, this, &RtspPlayerWidget::stopStream);

    initializeGStreamer();
}

RtspPlayerWidget::~RtspPlayerWidget()
{
    cleanupGStreamer();
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

void RtspPlayerWidget::startStream()
{
    QString rtspUrl = rtspUrlInput->text();
    if (rtspUrl.isEmpty()) {
        QMessageBox::warning(this, "Error", "RTSP URL cannot be empty.");
        return;
    }

    pipeline = gst_parse_launch(
        QString("playbin uri=%1 video-sink=xvimagesink").arg(rtspUrl).toUtf8().constData(), nullptr);

    if (!pipeline) {
        QMessageBox::critical(this, "Error", "Failed to create GStreamer pipeline.");
        return;
    }

    // Set the video widget as the rendering surface
    GstElement* videoSink = gst_bin_get_by_interface(GST_BIN(pipeline), GST_TYPE_VIDEO_OVERLAY);
    if (videoSink) {
        gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(videoSink),
                                            videoWidget->winId());
        gst_object_unref(videoSink);
    }

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
}

void RtspPlayerWidget::stopStream()
{
    if (pipeline) {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
        pipeline = nullptr;
    }
}
