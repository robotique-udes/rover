#ifndef GSTREAMERWORKER_HPP
#define GSTREAMERWORKER_HPP

#include <QObject>
#include <QString>
#include <gst/gst.h>

class GStreamerWorker : public QObject
{
    Q_OBJECT

  public:
    explicit GStreamerWorker(QObject* parent_ = nullptr);
    ~GStreamerWorker();
    GstElement* getPipeline() const
    {
        return _pipeline;
    }

    void setTargetId(const QString& id);
    QString getTargetId() const;

  public slots:
    void startPipeline(const QString& rtspUrl_);
    void stopPipeline();

  signals:
    void pipelineStarted(GstElement* pipeline_);
    void pipelineStopped();
    void errorOccurred(const QString& error_);
    void connectionFailed();
    void frameReceived();

  private:
    QString buildPipelineString(const QString& rtspUrl_) const;
    void cleanupGStreamer();

    GstElement* _pipeline = nullptr;
    QString _lastUrl;
    QString _targetId;

    gulong _newSampleSignalId = 0;
    int _consecutiveErrorsCount = 0;
    static constexpr int MAX_CONSECUTIVE_ERRORS = 3;
};

#endif