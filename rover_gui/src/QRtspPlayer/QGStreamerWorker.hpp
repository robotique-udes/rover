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
    void connectionFailed(); // New signal for permanent connection failure
    void frameReceived();

  private:
    QString buildPipelineString(const QString& rtspUrl) const;
    void cleanupGStreamer();

    gulong newSampleSignalId = 0;
    gulong padAddedSignalId = 0;
    gulong busErrorSignalId = 0;
    gulong busWarningSignalId = 0;
    
    GstElement* m_pipeline = nullptr;
    QString m_lastUrl; // Store the last URL for connection status tracking
};

#endif