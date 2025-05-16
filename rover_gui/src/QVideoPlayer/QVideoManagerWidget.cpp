#include "QVideoManagerWidget.hpp"
#include "QLogManager.hpp"
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

void QVideoManagerWidget::CB_updateArucoDetectionManager(void)
{
    if (_playerWorkerThread.get())
    {
        _playerWorkerThread->updateDetectionManager(_client_arucoDetectionManager);
    }
    else
    {
        UI_LOG_ERROR(ARUCO_DETECTION, "Error, couldn't access Video Player worker", "");
    }
}

void QVideoManagerWidget::CB_displayArucoDetected(rover_msgs::msg::Aruco msg_)
{
    std::string url = msg_.cam_url;
    std::vector<uint16_t> detectedIds = msg_.id;

    for (auto& widget : _videoPlaysWidgets)
    {
        if (widget && widget->getCamURL() == url)
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
            if (widget && widget->getCamURL() == url)
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
        std::string cameraUrl = "";
        if (i < CAMERA_NAME_ORDER.size()
            && Constants::CameraInfo::CAMERA_URL_MAP.find(CAMERA_NAME_ORDER[i]) != Constants::CameraInfo::CAMERA_URL_MAP.end())
        {
            cameraUrl = Constants::CameraInfo::CAMERA_URL_MAP.at(CAMERA_NAME_ORDER[i]);
        }
        else if (i < CAMERA_NAME_ORDER.size())
        {
            UI_LOG_WARNING(GENERAL,
                           QString("Couldn't find url for camera named %1 in camera infos.").arg(CAMERA_NAME_ORDER[i]),
                           "");
        }

        _videoPlaysWidgets[i] = std::make_unique<QVideoPlayerWidget>(_node, cameraUrl, i, _playerWorkerThread);
        _videoPlaysWidgets[i]->setObjectName(QString("camera%1_widget").arg(i + 1));
    }

    uint16_t index = 0;
    for (auto& widget : _videoPlaysWidgets)
    {
        if (widget)
        {
            int row = index / 3;
            int col = index % 3;
            _videoPlayerLayout.addWidget(widget.get(), row, col);
            index++;
        }
    }
}

void QVideoManagerWidget::initArucoPublisher(void)
{
    if (_node)
    {
        _sub_arucoDetection = _node->create_subscription<rover_msgs::msg::Aruco>(TOPIC_ARUCO_DETECTIONS,
                                                                                 5,
                                                                                 [this](const rover_msgs::msg::Aruco msg)
                                                                                 {
                                                                                     this->CB_displayArucoDetected(msg);
                                                                                 });
    }
    else
    {
        UI_LOG_ERROR(GENERAL, "Error, GUI node is invalid", "");
    }
}

void QVideoManagerWidget::initArucoClient(void)
{
    if (_node)
    {
        _client_arucoDetectionManager = _node->create_client<rover_msgs::srv::ArucoDetection>(SERVICE_ARUCO_NAME);
    }
    else
    {
        UI_LOG_ERROR(GENERAL, "Error, GUI node is invalid", "");
    }

    for (auto& widget : _videoPlaysWidgets)
    {
        if (widget)
        {
            widget->setArucoClientManager(_client_arucoDetectionManager);
        }
    }

    _timer_detectionManagerUpdate = _node->create_wall_timer(std::chrono::milliseconds(DELAY_DETECTION_MANAGER_UPDATE),
                                                             [this](void)
                                                             {
                                                                 this->CB_updateArucoDetectionManager();
                                                             });
}