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
    
    // Callback wrappers
    static GstFlowReturn onNewSampleCallback(GstElement* sink_, gpointer userData_);
    static void onGstErrorMessage(GstBus* bus_, GstMessage* msg_, gpointer userData_);
    static void onGstWarningMessage(GstBus* bus_, GstMessage* msg_, gpointer userData_);
    static void onDecodebinPadAdded(GstElement* decodebin_, GstPad* pad_, gpointer userData_);
    
    // Signal handler IDs for proper cleanup
    gulong _newSampleSignalId = 0;
    gulong _padAddedSignalId = 0;
    gulong _busErrorSignalId = 0;
    gulong _busWarningSignalId = 0;
    
    // Pipeline state
    GstElement* _pipeline = nullptr;
    QString _lastUrl;
    int _consecutiveErrorsCount = 0;
    static constexpr int MAX_CONSECUTIVE_ERRORS = 3;
};

#endif