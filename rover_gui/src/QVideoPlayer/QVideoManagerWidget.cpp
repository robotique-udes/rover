#include "QVideoManagerWidget.hpp"
#include <QString>

QVideoManagerWidget::QVideoManagerWidget(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_),
    _videoPlayerLayout(this),
    _playerWorkerThread(std::make_shared<QPlayerWorker>())

{
    for (size_t i = 0; i < NBR_CAM_TO_TRACK; ++i)
    {
        std::shared_ptr<QVideoPlayerWidget> widget = std::make_shared<QVideoPlayerWidget>(_node, this, _cameras_urls[i],i, _playerWorkerThread);
    
        widget->setObjectName(QString("camera%1_widget").arg(i+1));
    
        _videoPlaysWidgets[i] = widget;

        if(widget==nullptr)
        {
            qDebug()<<"wtf";
        }

        else
        {
            qDebug()<<"ok";
        }

    }

    uint16_t index = 0;
    for(auto& widget:_videoPlaysWidgets)
    {
        int row = index / 3;  // For example, 3 widgets per row
        int col = index % 3;  // Adjust this logic depending on the number of widgets per row
        _videoPlayerLayout.addWidget(widget.get(), row, col);
        index++;
    }

    connect(_playerWorkerThread.get(), &QPlayerWorker::urlFoundInDetection, this, &QVideoManagerWidget::onArucoDetectionIsLive);

    _client_arucoDetectionManager = _node->create_client<rover_msgs::srv::ArucoDetection>("/rover/auxiliary/aruco/manager");

    _sub_arucoDetection = _node->create_subscription<rover_msgs::msg::Aruco>("/rover/video/aruco",
                                                                       6,
                                                                       [this](const rover_msgs::msg::Aruco msg)
                                                                       {
                                                                           CB_displayArucoDetected(msg);
                                                                       });

    for(auto& widget:_videoPlaysWidgets)
    {
        widget->setArucoClientManager(_client_arucoDetectionManager);
    }

    this->setLayout(&_videoPlayerLayout);

    _timer_detectionManagerUpdate = _node->create_wall_timer(std::chrono::milliseconds(DELAY_DETECTION_MANAGER_UPDATE),
                                                             [this](void)
                                                             {
                                                                 this->CB_updateArucoDetectionManager();
                                                             });

    _playerWorkerThread->start();

    // Add your dashboard widget here
}

void QVideoManagerWidget::CB_updateArucoDetectionManager()
{
    if(_playerWorkerThread.get() != nullptr)
    {
        _playerWorkerThread->updateDetectionManager(_client_arucoDetectionManager);
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error, couldn't access Video Player worker");
    }
}

void QVideoManagerWidget::onArucoDetectionIsLive(std::vector<std::string> live_url_list_)
{
    for(auto& widget:_videoPlaysWidgets)
    {
        bool urlFound = false;
        for (const auto& url : live_url_list_)
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

void QVideoManagerWidget::CB_displayArucoDetected(rover_msgs::msg::Aruco msg_)
{
    std::string url = msg_.cam_url;
    std::vector<uint16_t> detectedIds = msg_.id;
    
    for(auto& widget:_videoPlaysWidgets)
    {
        if (widget != nullptr)
        {
            if(widget->getCamURL() == url)
            {
                widget->displayDetectedArucos(detectedIds);            
                emit widget->arucoCameraFailure(msg_.valid);
            }
        }
    }
}