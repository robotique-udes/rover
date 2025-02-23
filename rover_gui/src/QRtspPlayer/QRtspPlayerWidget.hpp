#ifndef RTSPPLAYERWIDGET_HPP
#define RTSPPLAYERWIDGET_HPP

#include <QLineEdit>
#include <QPushButton>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <gst/gst.h>

#include "QGStreamerWorker.hpp"
#include "UI_Player.h"

class RtspPlayerWidget : public QWidget
{
    Q_OBJECT

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
    QLineEdit* rtspUrlInput;
    QPushButton* startButton;
    QPushButton* stopButton;
    QWidget* videoWidget;

    void updateStatusIndicator(const QString& color);
};

#endif
