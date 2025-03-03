#ifndef GSTREAMERWORKER_HPP
#define GSTREAMERWORKER_HPP

#include <QObject>
#include <QString>
#include <gst/gst.h>

class GStreamerWorker : public QObject
{
    Q_OBJECT

  public:
    explicit GStreamerWorker(QObject* parent = nullptr);
    GstElement* getPipeline() const
    {
        return m_pipeline;
    }

    ~GStreamerWorker();

  public slots:
    void startPipeline(const QString& rtspUrl);
    void stopPipeline();

  signals:
    void pipelineStarted(GstElement* pipeline);
    void pipelineStopped();
    void errorOccurred(const QString& error);
    void frameReceived();

  private:
    QString buildPipelineString(const QString& rtspUrl) const;
    void cleanupGStreamer();

    GstElement* m_pipeline = nullptr;
};

#endif
