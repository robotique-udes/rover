#ifndef GSTREAMERWORKER_HPP
#define GSTREAMERWORKER_HPP

#include <QObject>
#include <gst/gst.h>
#include <QString>

class GStreamerWorker : public QObject
{
    Q_OBJECT

public:
    explicit GStreamerWorker(QObject* parent = nullptr);
    ~GStreamerWorker();

public slots:
    void startPipeline(const QString& rtspUrl);
    void stopPipeline();

signals:
    void pipelineStarted(GstElement* pipeline);
    void pipelineStopped();
    void errorOccurred(const QString& error);

private:
    GstElement* pipeline;
    void initializeGStreamer();
    void cleanupGStreamer();
};

#endif // GSTREAMERWORKER_HPP
