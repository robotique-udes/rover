#include "QVideoManagerWidget.hpp"
#include "QLogManager.hpp"
#include "rover_lib2/helpers/assert.hpp"
#include <QString>

using namespace LogUtils;

QVideoManagerWidget::QVideoManagerWidget(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_),
    _videoPlayerLayout(this),
    _playerWorkerThreadAruco(std::make_shared<QPlayerWorker>()),
    _playerWorkerThreadRecording(std::make_shared<QPlayerWorker>())
{
    this->initWidget();

    connect(_playerWorkerThreadAruco.get(),
            &QPlayerWorker::urlFoundInDetection,
            this,
            &QVideoManagerWidget::onArucoDetectionIsLive);
    connect(_playerWorkerThreadRecording.get(), &QPlayerWorker::setCursorWaiting, this, &QVideoManagerWidget::onSetCursorWaiting);

    for (size_t i = 0; i < NBR_CAM_TO_TRACK; ++i)
    {
        connect(_videoPlaysWidgets[i].get(),
                &QVideoPlayerWidget::notifyCameraAnglePublisher,
                this,
                &QVideoManagerWidget::CB_pubCameraAngle);
    }

    this->initArucoClient();
    this->initArucoPublisher();

    this->initCameraControlClient();
    this->initCameraControlSubscriber();
    this->initCameraAnglePublisher();

    this->setLayout(&_videoPlayerLayout);

    _playerWorkerThreadAruco->start();
    _playerWorkerThreadAruco->setThreadName("WorkerAruco");
    _playerWorkerThreadRecording->start();
    _playerWorkerThreadRecording->setThreadName("WorkerRecord");
}

void QVideoManagerWidget::CB_updateArucoDetectionManager(void)
{
    if (_playerWorkerThreadAruco.get())
    {
        _playerWorkerThreadAruco->updateDetectionManager(_client_arucoDetectionManager);
    }
    else
    {
        UI_LOG_ERROR(ARUCO_DETECTION, "Error, couldn't access Video Player worker", nullptr);
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
                           nullptr);
        }

        _videoPlaysWidgets[i]
            = std::make_unique<QVideoPlayerWidget>(_node, cameraUrl, i, _playerWorkerThreadAruco, _playerWorkerThreadRecording);
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
        UI_LOG_ERROR(GENERAL, "Error, GUI node is invalid", nullptr);
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
        UI_LOG_ERROR(GENERAL, "Error, GUI node is invalid", nullptr);
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

void QVideoManagerWidget::initCameraControlClient(void)
{
    if (_node)
    {
        _client_cameraControlManager = _node->create_client<rover_msgs::srv::CameraControl>(SERVICE_RECORDING_NAME);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, GUI node is invalid");
    }

    ASSERT_COND(_node != nullptr);
    for (auto& widget : _videoPlaysWidgets)
    {
        widget->setCameraControlClientManager(_client_cameraControlManager);
    }

    _timer_clientCameraControlHealth = _node->create_wall_timer(
        std::chrono::milliseconds(DELAY_DETECTION_MANAGER_UPDATE),
        [this](void)
        {
            bool available = _client_cameraControlManager->wait_for_service(std::chrono::milliseconds(TIMEOUT_SERVICE_AVAILABLE));
            for (auto& widget : _videoPlaysWidgets)
            {
                widget->CB_serviceCameraControlAvailable(available);
            }
        });
    return;
}

void QVideoManagerWidget::initCameraAnglePublisher(void)
{
    _pub_cameraAngle = _node->create_publisher<rover_msgs::msg::CameraControl>(CAMERA_ANGLE_CONTROL_TOPIC, QOS_DEFAULT);
}

void QVideoManagerWidget::initCameraControlSubscriber(void)
{
    _sub_cameraList = _node->create_subscription<rover_msgs::msg::CameraList>(TOPIC_RECORDING_INFO,
                                                                              1,
                                                                              [this](const rover_msgs::msg::CameraList msg)
                                                                              {
                                                                                  for (auto& widget : _videoPlaysWidgets)
                                                                                  {
                                                                                      widget->CB_cameraListUpdate(msg.urls);
                                                                                  }
                                                                              });
}

void QVideoManagerWidget::onSetCursorWaiting(bool waiting_)
{
    if (waiting_)
    {
        this->setCursor(Qt::WaitCursor);
    }
    else
    {
        this->setCursor(Qt::ArrowCursor);
    }
}

void QVideoManagerWidget::CB_pubCameraAngle(std::string camURL_, float pitch_)
{
    if (Constants::CameraInfo::CAMERA_URL_MAP.find("Main") == Constants::CameraInfo::CAMERA_URL_MAP.end()
        || Constants::CameraInfo::CAMERA_URL_MAP.find("Antenna") == Constants::CameraInfo::CAMERA_URL_MAP.end())
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Can't publish camera angles. Coulnd't find 'Main' or 'Antenna' in camera map!");
        return;
    }

    rover_msgs::msg::CameraControl msg;

    if (camURL_ == Constants::CameraInfo::CAMERA_URL_MAP.at("Main"))
    {
        msg.id_cam = rover_msgs::msg::CameraControl::ID_CAM_MAIN;
    }
    else if (camURL_ == Constants::CameraInfo::CAMERA_URL_MAP.at("Antenna"))
    {
        msg.id_cam = rover_msgs::msg::CameraControl::ID_CAM_ANTENNA;
    }
    else
    {
        return;
    }

    msg.pitch = pitch_;
    msg.yaw = 0.0f;

    _pub_cameraAngle->publish(msg);
}