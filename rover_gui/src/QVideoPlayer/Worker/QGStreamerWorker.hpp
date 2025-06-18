#ifndef GSTREAMERWORKER_HPP
#define GSTREAMERWORKER_HPP

#include <QObject>
#include <QString>
#include <QWidget>
#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>

class GStreamerWorker : public QObject
{
    Q_OBJECT

  public:
    explicit GStreamerWorker(QObject* parent_ = nullptr);
    ~GStreamerWorker();

    void setTargetWidget(QWidget* target_widget_);
    void setVideoWidget(QWidget* video_widget_);
    void setupVideoOverlay();
    void pausePipeline();
    void stopPipeline();

  public slots:
    void startPipeline(const QString& rtsp_url_);

  signals:
    void pipelineStarted(GstElement* pipeline_);
    void pipelineStopped();
    void errorOccurred(const QString& error_message_);
    void frameReceived();
    void connectionFailed();

  private:
    std::string buildPipelineString(const std::string& rtsp_url_) const;
    void cleanupGStreamer();

    QWidget* _video_widget = nullptr;
    QWidget* _target_widget = nullptr;

    GstElement* _pipeline = nullptr;
    QString _last_rtsp_url;

    gulong _new_sample_signal_id = 0;
    gulong _error_handler_id = 0;

    static void on_gst_error_message(GstBus* bus_, GstMessage* message_, gpointer user_data_);
    static GstFlowReturn on_new_sample(GstElement* sink_, gpointer user_data_);
};

#endif