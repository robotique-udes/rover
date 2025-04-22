#include "QVideoManagerWidget.hpp"
#include <QString>

QVideoManagerWidget::QVideoManagerWidget(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_),
    _videoPlayerLayout(this),
    _playerWorkerThread(std::make_shared<QPlayerWorker>())

{
    this->initWidget();

    connect(_playerWorkerThread.get(), &QPlayerWorker::urlFoundInDetection, this, &QVideoManagerWidget::onArucoDetectionIsLive);

    this->initArucoClient();
    this->initArucoPublisher();

    this->setLayout(&_videoPlayerLayout);

    _playerWorkerThread->start();
}

void QVideoManagerWidget::CB_updateArucoDetectionManager()
{
    if (_playerWorkerThread.get())
    {
        _playerWorkerThread->updateDetectionManager(_client_arucoDetectionManager);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, couldn't access Video Player worker");
    }
}

void QVideoManagerWidget::CB_displayArucoDetected(rover_msgs::msg::Aruco msg_)
{
    std::string url = msg_.cam_url;
    std::vector<uint16_t> detectedIds = msg_.id;

    for (auto& widget : _videoPlaysWidgets)
    {
        if (widget != nullptr && widget->getCamURL() == url)
        {
            widget->displayDetectedArucos(detectedIds);
            emit widget->arucoCameraFailure(msg_.valid);
        }
    }
}

void QVideoManagerWidget::onArucoDetectionIsLive(std::vector<std::string> liveUrlList_)
{
    for (auto& widget : _videoPlaysWidgets)
    {
        bool urlFound = false;
        for (const auto& url : liveUrlList_)
        {
            if (widget->getCamURL() == url)
            {
                urlFound = true;
                break;
            }
        }
        widget->arucoStillAliveUpdate(urlFound);
    }
}

void QVideoManagerWidget::initWidget(void)
{
    for (size_t i = 0UL; i < NBR_CAM_TO_TRACK; ++i)
    {
        std::shared_ptr<QVideoPlayerWidget> widget
            = std::make_shared<QVideoPlayerWidget>(_node, this, _cameras_urls[i], i, _playerWorkerThread);

        widget->setObjectName(QString("camera%1_widget").arg(i + 1));

        _videoPlaysWidgets[i] = widget;
    }

    uint16_t index = 0;
    for (auto& widget : _videoPlaysWidgets)
    {
        int row = index / 3;
        int col = index % 3;
        _videoPlayerLayout.addWidget(widget.get(), row, col);
        index++;
    }
}

void QVideoManagerWidget::initArucoPublisher(void)
{
    if (_node)
    {
        _sub_arucoDetection = _node->create_subscription<rover_msgs::msg::Aruco>("/rover/video/aruco",
                                                                                 5,
                                                                                 [this](const rover_msgs::msg::Aruco msg)
                                                                                 {
                                                                                     CB_displayArucoDetected(msg);
                                                                                 });
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, GUI node is invalid");
    }
}

void QVideoManagerWidget::initArucoClient(void)
{
    if (_node)
    {
        _client_arucoDetectionManager = _node->create_client<rover_msgs::srv::ArucoDetection>("/rover/auxiliary/aruco/manager");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, GUI node is invalid");
    }

    for (auto& widget : _videoPlaysWidgets)
    {
        widget->setArucoClientManager(_client_arucoDetectionManager);
    }

    _timer_detectionManagerUpdate = _node->create_wall_timer(std::chrono::milliseconds(DELAY_DETECTION_MANAGER_UPDATE),
                                                             [this](void)
                                                             {
                                                                 this->CB_updateArucoDetectionManager();
                                                             });
}