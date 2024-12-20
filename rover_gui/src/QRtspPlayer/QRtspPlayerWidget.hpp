#ifndef RTSPPLAYERWIDGET_HPP
#define RTSPPLAYERWIDGET_HPP

#include <QWidget>
#include <QThread>
#include "QGStreamerWorker.hpp"
#include <gst/gst.h>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include "UI_Player.h"

class RtspPlayerWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RtspPlayerWidget(QWidget* parent = nullptr);
    void startStream(const QString& rtspUrl);
    void stopStream();
    ~RtspPlayerWidget();

private slots:
    void onPipelineStarted(GstElement* pipeline);
    void onErrorOccurred(const QString& error);

signals:
    void requestStartStream(const QString& rtspUrl); 
    void requestStopStream();

private:
    Ui::RtspPlayerWidget* ui;
    GstElement* pipeline;
    QLineEdit* rtspUrlInput;
    QPushButton* startButton;
    QPushButton* stopButton;
    QWidget* videoWidget;
    QThread* workerThread;
    GStreamerWorker* gstreamerWorker;
};

#endif 
