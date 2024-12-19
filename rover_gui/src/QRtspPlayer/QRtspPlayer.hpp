#ifndef RTSPPLAYERWIDGET_HPP
#define RTSPPLAYERWIDGET_HPP

#include <QWidget>
#include <gst/gst.h>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>

class RtspPlayerWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RtspPlayerWidget(QWidget* parent = nullptr);
    ~RtspPlayerWidget();

public slots:
    void startStream();
    void stopStream();

private:
    GstElement* pipeline;
    QLineEdit* rtspUrlInput;
    QPushButton* startButton;
    QPushButton* stopButton;
    QWidget* videoWidget;

    void initializeGStreamer();
    void cleanupGStreamer();
};

#endif 
