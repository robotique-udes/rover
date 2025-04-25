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
    
    // Access to pipeline for callbacks
    GstElement* getPipeline() const { return _pipeline; }

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
    // Helper methods
    QString buildPipelineString(const QString& rtspUrl_) const;
    void cleanupGStreamer();
    
    // Pipeline state - can't convert these to stack allocation because they're C-style pointers
    GstElement* _pipeline = nullptr;
    QString _lastUrl;
    
    // Signal handler IDs for proper cleanup
    gulong _newSampleSignalId = 0;
    int _consecutiveErrorsCount = 0;
    static constexpr int MAX_CONSECUTIVE_ERRORS = 3;
};

#endif