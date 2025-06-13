#ifndef QVIDEOPLAYER_QVIDEORECORDERWIDGET_HPP
#define QVIDEOPLAYER_QVIDEORECORDERWIDGET_HPP

#include "rclcpp/rclcpp.hpp"
#include <QtWidgets/QWidget>

#include "QVideoPlayerWidget.hpp"
#include "UI_VideoPlayer.h"

class QVideoRecorderWidget : public QWidget
{
    Q_OBJECT

  public:
    QVideoRecorderWidget(std::shared_ptr<rclcpp::Node> guiNode_,
                       std::string url_,
                       uint16_t tag_,
                       std::shared_ptr<QRecordingWorker> workerThreadRecording_);

    ~QVideoRecorderWidget();
};

#endif  // QVIDEOPLAYER_QVIDEORECORDERWIDGET_HPP