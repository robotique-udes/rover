#ifndef RTSPPLAYERWIDGET_HPP
#define RTSPPLAYERWIDGET_HPP

#include <QThread>
#include <QTimer>
#include <QWidget>
#include <gst/gst.h>

#include "QGStreamerWorker.hpp"
#include "UI_Player.h"

class RtspPlayerWidget : public QWidget
{
    Q_OBJECT
    static constexpr int RECONNECT_INTERVAL = 5000;

  public:
    explicit RtspPlayerWidget(QWidget* parent = nullptr);
    ~RtspPlayerWidget();

    void startStream(const QString& rtspUrl);
    void stopStream();

  private slots:
    void onPipelineStarted(GstElement* pipeline);
    void onErrorOccurred(const QString& error);

  signals:
    void requestStartStream(const QString& rtspUrl);
    void requestStopStream();

  private:
    Ui::RtspPlayerWidget* ui;

    QThread* workerThread;
    GStreamerWorker* gstreamerWorker;

    QTimer* reconnectTimer;
    QTimer* frameTimeoutTimer;

    GstElement* pipeline;
    bool receivingFrames;
    bool inReconnectionMode;

    void updateStatusIndicator(const QString& color);
};

#endif