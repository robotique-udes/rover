#include "QVideoManagerWidget.hpp"
#include "QLogManager.hpp"
#include "rover_lib2/helpers/assert.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include <QString>
#include <utility>

using namespace LogUtils;

QVideoManagerWidget::QVideoManagerWidget(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_),
    _cameraInterface(guiNode_, CAMERA_PTZ_CMD_TOPIC_GUI, CAMERA_PTZ_CONFIG_TOPIC_GUI, CAMERA_POWER_TOPIC_GUI),
    _playerWorkerThreadAruco(std::make_shared<QPlayerWorker>()),
    _panoramaWorkerThread(std::make_shared<QPanoramaWorker>()),
    _tabWidget(this),
    _gridContainer(nullptr),
    _vSubLayoutContainer(nullptr),
    _altLayoutContainer(nullptr),
    _arm3LayoutContainer(nullptr),
    _arm4LayoutContainer(nullptr)
{
    this->initWidget();

    _resetLayout_PB.setIcon(QIcon(":/icons/refresh.png"));
    _tabWidget.setCornerWidget(&_resetLayout_PB, Qt::TopRightCorner);

    connect(&_resetLayout_PB, &QPushButton::clicked, this, &QVideoManagerWidget::setSplitterInitialGeometry);

    connect(_playerWorkerThreadAruco.get(),
            &QPlayerWorker::urlFoundInDetection,
            this,
            &QVideoManagerWidget::onArucoDetectionIsLive);

    for (size_t i = 0; i < NBR_CAM_TO_TRACK; ++i)
    {
        connect(_videoPlaysWidgets[i].get(), &QVideoPlayerWidget::updatePTZCmd, this, &QVideoManagerWidget::setPTZCmd);
        connect(_playerWorkerThreadRecording[i].get(),
                &QRecordingWorker::setCursorWaiting,
                this,
                &QVideoManagerWidget::onSetCursorWaiting);
    }

    connect(&_tabWidget, &QTabWidget::currentChanged, this, &QVideoManagerWidget::onTabChanged);

    this->initArucoClient();
    this->initArucoPublisher();

    this->initCameraControlClient();
    this->initCameraListSubscriber();
    this->initCameraStatusSubscriber();

    _gridContainer.setLayout(&_gridLayout);
    _altLayoutContainer.setLayout(&_altLayout);
    _vSubLayoutContainer.setLayout(&_vSubLayout);
    _arm3LayoutContainer.setLayout(&_arm3Layout);
    _arm3SubLayoutContainer.setLayout(&_arm3SubLayout);
    _arm4LayoutContainer.setLayout(&_arm4Layout);

    _mainLayout.addWidget(&_tabWidget);
    this->setLayout(&_mainLayout);

    _altLayout.addWidget(&_splitter);
    _arm3Layout.addWidget(&_arm3Splitter);
    _tabWidget.addTab(&_gridContainer, "grid");
    _tabWidget.addTab(&_altLayoutContainer, "alt");
    _tabWidget.addTab(&_arm3LayoutContainer, "arm3");
    _tabWidget.addTab(&_arm4LayoutContainer, "arm4");
    _tabWidget.setCurrentIndex(std::to_underlying(eTabIndex::ALT));

    this->initPanoramaClient();
    this->initCameraInterface();

    _playerWorkerThreadAruco->start();
    _playerWorkerThreadAruco->setThreadName("WorkerAruco");
    _panoramaWorkerThread->start();
    _panoramaWorkerThread->setThreadName("QWorkerPano");
}

QVideoManagerWidget::~QVideoManagerWidget()
{
    rover_msgs::msg::CameraControl powerMsg;
    powerMsg.power_on = false;
    for (size_t id = 0; id < std::to_underlying(Constants::CameraInfo::eCamNames::eLast); ++id)
    {
        powerMsg.id_cam = id;
        _cameraInterface.setPowerCmd(powerMsg, static_cast<Constants::CameraInfo::eCamNames>(id));
    }
}

void QVideoManagerWidget::onTabChanged(uint16_t index_)
{
    if (index_ == std::to_underlying(eTabIndex::GRID))
    {
        uint16_t index = 0;
        for (const auto& widget : _videoPlaysWidgets)
        {
            if (widget)
            {
                int row = index / 3;
                int col = index % 3;
                _gridLayout.addWidget(widget.get(), row, col);
                index++;
            }
        }
        _resetLayout_PB.setVisible(false);
    }
    else if (index_ == std::to_underlying(eTabIndex::ALT))
    {
        if (_videoPlaysWidgets[1])
        {
            _vSubLayout.addWidget(_videoPlaysWidgets[1].get());
        }

        if (_videoPlaysWidgets[2])
        {
            _vSubLayout.addWidget(_videoPlaysWidgets[2].get());
        }

        _splitter.addWidget(&_vSubLayoutContainer);

        if (_videoPlaysWidgets[0])
        {
            _splitter.insertWidget(0, _videoPlaysWidgets[0].get());
        }
        _resetLayout_PB.setVisible(true);
        this->setSplitterInitialGeometry();
    }
    else if (index_ == std::to_underlying(eTabIndex::ARM3))
    {
        if (_videoPlaysWidgets[std::to_underlying(Constants::CameraInfo::eCamNames::ARM_TOP)])
        {
            _arm3SubLayout.addWidget(_videoPlaysWidgets[std::to_underlying(Constants::CameraInfo::eCamNames::ARM_TOP)].get());
        }

        if (_videoPlaysWidgets[std::to_underlying(Constants::CameraInfo::eCamNames::ANTENNA)])
        {
            _arm3SubLayout.addWidget(_videoPlaysWidgets[std::to_underlying(Constants::CameraInfo::eCamNames::ANTENNA)].get());
        }

        _arm3Splitter.addWidget(&_arm3SubLayoutContainer);

        if (_videoPlaysWidgets[std::to_underlying(Constants::CameraInfo::eCamNames::ARM_SIDE)])
        {
            _arm3Splitter.insertWidget(0,
                                       _videoPlaysWidgets[std::to_underlying(Constants::CameraInfo::eCamNames::ARM_SIDE)].get());
        }
        this->setSplitterInitialGeometry();
        _resetLayout_PB.setVisible(true);
    }
    else if (index_ == std::to_underlying(eTabIndex::ARM4))
    {
        if (_videoPlaysWidgets[std::to_underlying(Constants::CameraInfo::eCamNames::ARM_SIDE)])
        {
            _arm4Layout.addWidget(_videoPlaysWidgets[std::to_underlying(Constants::CameraInfo::eCamNames::ARM_SIDE)].get(), 1, 0);
        }

        if (_videoPlaysWidgets[std::to_underlying(Constants::CameraInfo::eCamNames::ARM_TOP)])
        {
            _arm4Layout.addWidget(_videoPlaysWidgets[std::to_underlying(Constants::CameraInfo::eCamNames::ARM_TOP)].get(), 1, 1);
        }

        if (_videoPlaysWidgets[std::to_underlying(Constants::CameraInfo::eCamNames::ANTENNA)])
        {
            _arm4Layout.addWidget(_videoPlaysWidgets[std::to_underlying(Constants::CameraInfo::eCamNames::ANTENNA)].get(), 0, 1);
        }

        if (_videoPlaysWidgets[std::to_underlying(Constants::CameraInfo::eCamNames::MAIN)])
        {
            _arm4Layout.addWidget(_videoPlaysWidgets[std::to_underlying(Constants::CameraInfo::eCamNames::MAIN)].get(), 0, 0);
        }
        _resetLayout_PB.setVisible(false);
    }
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
            emit widget->displayDetectedArucos(detectedIds);
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
        if (i < CAMERA_NAME_ORDER.size())
        {
            if (i >= std::to_underlying(Constants::CameraInfo::eCamNames::eLast))
            {
                cameraUrl = "";
            }
            else
            {
                cameraUrl = Constants::CameraInfo::CAMERA_INFO[i][std::to_underlying(Constants::CameraInfo::eInfoType::URL)];
            }
        }
        else if (i < CAMERA_NAME_ORDER.size())
        {
            UI_LOG_WARNING(GENERAL,
                           QString("Couldn't find url for camera named %1 in camera infos.").arg(CAMERA_NAME_ORDER[i]),
                           nullptr);
        }

        _playerWorkerThreadRecording[i] = std::make_shared<QRecordingWorker>();
        _playerWorkerThreadRecording[i]->start();
        _playerWorkerThreadRecording[i]->setThreadName("WorkerRecord" + std::to_string(i));

        _videoPlaysWidgets[i] = std::make_unique<QVideoPlayerWidget>(_node,
                                                                     cameraUrl,
                                                                     i,
                                                                     _playerWorkerThreadAruco,
                                                                     _playerWorkerThreadRecording[i],
                                                                     _panoramaWorkerThread);
        _videoPlaysWidgets[i]->setObjectName(QString("camera%1_widget").arg(i + 1));
    }

    uint16_t index = 0;
    for (auto& widget : _videoPlaysWidgets)
    {
        if (widget)
        {
            int row = index / 3;
            int col = index % 3;
            _gridLayout.addWidget(widget.get(), row, col);
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
        _client_cameraIR = _node->create_client<rover_msgs::srv::CameraIR>(SERVICE_IR);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, GUI node is invalid");
    }

    ASSERT_COND(_node != nullptr);
    for (auto& widget : _videoPlaysWidgets)
    {
        widget->setCameraControlClientManager(_client_cameraControlManager);
        widget->setCameraIRClient(_client_cameraIR);
    }

    _timer_clientCameraControlHealth = _node->create_wall_timer(
        std::chrono::milliseconds(DELAY_DETECTION_MANAGER_UPDATE),
        [this](void)
        {
            bool available = _client_cameraControlManager->wait_for_service(std::chrono::milliseconds(TIMEOUT_SERVICE_AVAILABLE));
            for (auto& widget : _videoPlaysWidgets)
            {
                widget->CB_srvCameraAvailable(available);
            }
        });
    return;
}

void QVideoManagerWidget::initCameraListSubscriber(void)
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

void QVideoManagerWidget::setSplitterInitialGeometry(void)
{
    uint16_t index = _tabWidget.currentIndex();
    if (index == std::to_underlying(eTabIndex::ALT))
    {
        int total = _splitter.width();
        int left = static_cast<int>(ALT_CAM_LAYOUT_PROPORTION * total);
        int right = total - left;
        _splitter.setSizes(QList<int>({left, right}));
    }
    else if (index == std::to_underlying(eTabIndex::ARM3))
    {
        int total = _arm3Splitter.width();
        int left = static_cast<int>(ALT_CAM_LAYOUT_PROPORTION * total);
        int right = total - left;
        _arm3Splitter.setSizes(QList<int>({left, right}));
    }
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

void QVideoManagerWidget::setPTZCmd(float yaw_, size_t id_)
{
    rover_msgs::msg::CameraControl msg;
    msg.id_cam = id_;
    msg.yaw = degToRad(yaw_);
    msg.power_on = true;
    msg.pitch = 0.0F;
    _cameraInterface.setPTZCmd(msg, static_cast<Constants::CameraInfo::eCamNames>(id_));
}

void QVideoManagerWidget::initPanoramaClient(void)
{
    if (_node)
    {
        _client_panoramique = _node->create_client<rover_msgs::srv::Panorama>(SERVICE_PANORAMA_NAME);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, GUI node is invalid");
    }

    ASSERT_COND(_node != nullptr);
    for (const std::unique_ptr<QVideoPlayerWidget>& widget : _videoPlaysWidgets)
    {
        widget->setPanoramaClientManager(_client_panoramique);
    }
}

void QVideoManagerWidget::initCameraStatusSubscriber(void)
{
    _sub_cameraStatus = _node->create_subscription<rover_msgs::msg::CameraControl>(
        CAMERA_STATUS_TOPIC,
        QOS_DEFAULT,
        [this](const rover_msgs::msg::CameraControl msg)
        {
            if (msg.id_cam >= std::to_underlying(Constants::CameraInfo::eCamNames::eLast))
            {
                QHelper::QToastNotification::getInstance().notifyFromAnyThread(
                    "Invalid message was received from /rover/camera/PTZ_status",
                    "cam_id was out of bound",
                    QHelper::QToastNotification::eNotifType::ERROR);
                return;
            }
            for (const std::unique_ptr<QVideoPlayerWidget>& widget : _videoPlaysWidgets)
            {
                emit widget->updateActualAngle(Constants::CameraInfo::CAMERA_INFO[static_cast<size_t>(
                                                   msg.id_cam)][std::to_underlying(Constants::CameraInfo::eInfoType::URL)],
                                               msg.yaw);
            }
        });
}

void QVideoManagerWidget::initCameraInterface(void)
{
    rover_msgs::msg::CameraConfig configMsg;
    configMsg.pan_max_position = CAMERA_MAX_ANGLE;
    configMsg.pan_min_position = CAMERA_MIN_ANGLE;
    configMsg.pan_max_speed = CAMERA_MAX_SPEED;
    configMsg.tilt_max_position = CAMERA_MAX_ANGLE;
    configMsg.tilt_min_position = CAMERA_MIN_ANGLE;
    configMsg.tilt_max_speed = CAMERA_MAX_SPEED;

    rover_msgs::msg::CameraControl cmdMsg;
    cmdMsg.yaw = CAMERA_CENTER_ANGLE;
    cmdMsg.pitch = 0.0F;
    cmdMsg.power_on = true;

    cmdMsg.id_cam = static_cast<uint8_t>(std::to_underlying(Constants::CameraInfo::eCamNames::MAIN));
    _cameraInterface.setPTZCmd(cmdMsg, Constants::CameraInfo::eCamNames::MAIN);
    configMsg.id_cam = static_cast<uint8_t>(std::to_underlying(Constants::CameraInfo::eCamNames::MAIN));
    _cameraInterface.setPTZConfig(configMsg, Constants::CameraInfo::eCamNames::MAIN);

    cmdMsg.id_cam = static_cast<uint8_t>(std::to_underlying(Constants::CameraInfo::eCamNames::ANTENNA));
    _cameraInterface.setPTZCmd(cmdMsg, Constants::CameraInfo::eCamNames::ANTENNA);
    configMsg.id_cam = static_cast<uint8_t>(std::to_underlying(Constants::CameraInfo::eCamNames::ANTENNA));
    _cameraInterface.setPTZConfig(configMsg, Constants::CameraInfo::eCamNames::ANTENNA);

    rover_msgs::msg::CameraControl powerMsg;
    powerMsg.power_on = true;

    for (size_t id = 0; id < std::to_underlying(Constants::CameraInfo::eCamNames::eLast); ++id)
    {
        powerMsg.id_cam = id;
        _cameraInterface.setPowerCmd(powerMsg, static_cast<Constants::CameraInfo::eCamNames>(id));
    }
}