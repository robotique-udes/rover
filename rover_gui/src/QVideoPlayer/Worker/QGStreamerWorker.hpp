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
    mutable std::mutex _pipelineMutex;
    QString buildPipelineString(const QString& rtspUrl_) const;
    void cleanupGStreamer();

    GstElement* _pipeline = nullptr;
    QString _lastUrl;

    QWidget* _targetWidget = nullptr;

    gulong _newSampleSignalId = 0;
    int _consecutiveErrorsCount = 0;
    static constexpr int MAX_CONSECUTIVE_ERRORS = 3;

    static void on_gst_error_message(GstBus* bus_, GstMessage* msg_, gpointer user_data_);
    static GstFlowReturn on_new_sample(GstElement* sink_, gpointer user_data_);
    static void on_gst_warning_message(GstBus* bus_, GstMessage* msg_, gpointer user_data_);

    // Bus signal handler IDs
    gulong _errorHandlerId = 0;
    gulong _warningHandlerId = 0;
    gulong _messageHandlerId = 0;
};

#endif