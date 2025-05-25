#ifndef GSTREAMERWORKER_HPP
#define GSTREAMERWORKER_HPP

#include <QObject>
#include <QString>
#include <QWidget>
#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>
#include <mutex>

class GStreamerWorker : public QObject
{
    Q_OBJECT

  public:
    explicit GStreamerWorker(QObject* parent_ = nullptr);
    ~GStreamerWorker();
    GstElement* getPipeline();

    void setTargetWidget(QWidget* widget);
    QWidget* getTargetWidget() const;
    void pausePipeline();
    void resumePipeline();
    void stopPipeline();

  public slots:
    void startPipeline(const QString& rtspUrl_);

  signals:
    void pipelineStarted(GstElement* pipeline_);
    void pipelineStopped();
    void errorOccurred(const QString& error_);
    void frameReceived();
    void connectionFailed();

  private:
    mutable std::mutex _pipelineMutex;
    std::string buildPipelineString(const std::string& rtspUrl_) const;
    void cleanupGStreamer();

    GstElement* _pipeline = nullptr;
    QString _lastUrl;

    QWidget* _targetWidget = nullptr;

    gulong _newSampleSignalId = 0;

    static void on_gst_error_message(GstBus* bus_, GstMessage* msg_, gpointer user_data_);
    static GstFlowReturn on_new_sample(GstElement* sink_, gpointer user_data_);

    gulong _errorHandlerId = 0;
    gulong _messageHandlerId = 0;
};

#endif