#include "QDeviceStatus.hpp"
#include <QStyle>

constexpr const char* STATUS_DEFAULT = "QWidget {"
                                       "background-color: #3c3f41;"
                                       "border-radius: 5px;"
                                       "padding: 5px 10px;"
                                       "}";

constexpr const char* STATUS_SUCCESS = "QWidget {"
                                       "background-color: #81c784;"
                                       "color: black;"
                                       "border-radius: 5px;"
                                       "padding: 5px 10px;"
                                       "}";

constexpr const char* STATUS_WARNING = "QWidget {"
                                       "background-color : #ffb74d;"
                                       "color: black;"
                                       "border-radius: 5px;"
                                       "padding: 5px 10px;"
                                       "}";

constexpr const char* STATUS_ERROR = "QWidget {"
                                     "background-color : #e57373;"
                                     "color: black;"
                                     "border-radius: 5px;"
                                     "padding: 5px 10px;"
                                     "}";

QDeviceStatus::QDeviceStatus(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_),
    _QStatusWorker(true, this)
{
    if (!_node)
    {
        throw std::invalid_argument("QDeviceStatus requires a valid ROS node");
    }

    _ui.setupUi(this);

    _deviceInfo[RoverCan2::Constant::eDeviceId::FRONTLEFT_MOTOR] = _ui.frontleftMotor;
    _deviceInfo[RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR] = _ui.frontrightMotor;
    _deviceInfo[RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR] = _ui.rearleftMotor;
    _deviceInfo[RoverCan2::Constant::eDeviceId::REARRIGHT_MOTOR] = _ui.rearrightMotor;
    _deviceInfo[RoverCan2::Constant::eDeviceId::GNSS] = _ui.gnss;
    _deviceInfo[RoverCan2::Constant::eDeviceId::DDB_CONTROLLER] = _ui.ddbController;
    _deviceInfo[RoverCan2::Constant::eDeviceId::CAMERA_ROVER_MAIN] = _ui.cameraMain;
    _deviceInfo[RoverCan2::Constant::eDeviceId::CAMERA_ROVER_ANTENNA] = _ui.cameraAntenne;
    _deviceInfo[RoverCan2::Constant::eDeviceId::LIGHTS_MAIN] = _ui.lightsMain;
    // _deviceInfo[RoverCan2::Constant::eDeviceId::SWITCHETH0] = _ui.switchETH0;
    // _deviceInfo[RoverCan2::Constant::eDeviceId::SWITCHETH1] = _ui.switchETH1;

    _deviceReboot[RoverCan2::Constant::eDeviceId::FRONTLEFT_MOTOR] = _ui.frontleftMotorInfo;
    _deviceReboot[RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR] = _ui.frontrightMotorInfo;
    _deviceReboot[RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR] = _ui.rearleftMotorInfo;
    _deviceReboot[RoverCan2::Constant::eDeviceId::REARRIGHT_MOTOR] = _ui.rearrightMotorInfo;
    _deviceReboot[RoverCan2::Constant::eDeviceId::GNSS] = _ui.gnssInfo;
    _deviceReboot[RoverCan2::Constant::eDeviceId::DDB_CONTROLLER] = _ui.ddbControllerInfo;
    _deviceReboot[RoverCan2::Constant::eDeviceId::CAMERA_ROVER_MAIN] = _ui.cameraMainInfo;
    _deviceReboot[RoverCan2::Constant::eDeviceId::CAMERA_ROVER_ANTENNA] = _ui.cameraAntenneInfo;
    _deviceReboot[RoverCan2::Constant::eDeviceId::LIGHTS_MAIN] = _ui.lightsMainInfo;
    // _deviceReboot[RoverCan2::Constant::eDeviceId::SWITCHETH0] = _ui.switchETH0Info;
    // _deviceReboot[RoverCan2::Constant::eDeviceId::SWITCHETH1] = _ui.switchETH1Info;

    _sub_deviceStatus
        = _node->create_subscription<rover_msgs::msg::CanDeviceStatus>("/rover/can/devices_status",
                                                                       QOS_DEFAULT,
                                                                       [this](const rover_msgs::msg::CanDeviceStatus& msg)
                                                                       {
                                                                           QMetaObject::invokeMethod(
                                                                               this,
                                                                               [this, msg]()
                                                                               {
                                                                                   this->callbackDeviceInfos(msg);
                                                                               },
                                                                               Qt::QueuedConnection);
                                                                       });

    _client_requestErrorStatus = _node->create_client<rover_msgs::srv::Empty>("/rover/can/request_error_state");
    this->updateDeviceInfo();

    connect(_ui.pb_serviceCall,
            &QPushButton::clicked,
            this,
            [this]()
            {
                this->setDefaultStyle();
                this->updateDeviceInfo();
            });

    connect(&_QStatusWorker,
            &QStatusWorker::onRequestDeviceStatusSuccessful,
            this,
            &QDeviceStatus::onRequestDeviceStatusSuccessful);
}

/**
 * @brief Makes the service call to request the error state of the devices.
 *
 * @param request_
 */
void QDeviceStatus::updateDeviceInfo()
{
    _QStatusWorker.requestDeviceStatusManager(_client_requestErrorStatus);
}

/**
 * @brief Handles the response from the service call
 *
 * @param success_
 * @param response_
 */
void QDeviceStatus::onRequestDeviceStatusSuccessful(bool success_, const std::string& response_)
{
    if (success_)
    {
        _numberOfCalls++;
        RCLCPP_INFO(rclcpp::get_logger("GUI"), "Service request succeeded: %s", response_.c_str());
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Service request failed: %s", response_.c_str());
    }
}

/**
 * @brief Starts updating the devices' informations when there is a new message.
 *
 * @param msg_
 */
void QDeviceStatus::callbackDeviceInfos(const rover_msgs::msg::CanDeviceStatus& msg_)
{
    RoverCan2::Constant::eDeviceId deviceID = RoverCan2::Constant::eDeviceId(msg_.id);

    _deviceMessageCount[deviceID]++;

    this->updateRebootCounter(deviceID);
    this->updateDeviceColor(deviceID, msg_);
    this->setStatusReport(deviceID);
}

/**
 * @brief Keeps track of the number of reboots for each device. Logs a message when a device reboots.
 *
 * @param deviceID_
 */
void QDeviceStatus::updateRebootCounter(RoverCan2::Constant::eDeviceId deviceID_)
{
    if (!_deviceInfo.contains(deviceID_) || !_deviceReboot.contains(deviceID_))
    {
        return;
    }

    uint16_t& deviceReboots = _numberOfDeviceReboots[deviceID_];
    uint16_t& oldDeviceReboots = _oldDeviceReboots[deviceID_];
    uint16_t& deviceRebootsFromButton = _numberOfDeviceRebootsFromButton[deviceID_];
    uint16_t& deviceMessageCount = _deviceMessageCount[deviceID_];

    if (_numberOfCalls < deviceMessageCount)
    {
        oldDeviceReboots = 0;
        deviceReboots = 0;
        return;
    }

    deviceReboots = deviceRebootsFromButton + _numberOfCalls - deviceMessageCount;

    if (deviceReboots != oldDeviceReboots)
    {
        oldDeviceReboots = deviceReboots;
        uint16_t deviceID = static_cast<uint16_t>(deviceID_);
        RCLCPP_INFO(rclcpp::get_logger("GUI"), "Device %d has rebooted since last call", deviceID);
    }
}

/**
 * @brief Updates the label text with the updated status report for the received device.
 *
 * @param deviceID_
 */
void QDeviceStatus::setStatusReport(RoverCan2::Constant::eDeviceId deviceID_)
{
    if (!_deviceInfo.contains(deviceID_) || !_deviceReboot.contains(deviceID_))
    {
        return;
    }

    QLabel* label = _deviceReboot[deviceID_];

    std::string deviceName = this->getDeviceName(deviceID_);
    uint16_t deviceID = static_cast<uint16_t>(deviceID_);

    QString labelText = QString::fromStdString(deviceName) + QString("\n\nID: 0x%1").arg(deviceID, 3, 16, QChar('0'))
                        + "\n\nReboots: " + QString::number(_numberOfDeviceReboots[deviceID_]);

    label->setText(labelText);
}

/**
 * @brief Updates the label color based on the device status. If ok: green, if warning: yellow, if error: red.
 *
 * @param deviceID_
 * @param deviceStatus_
 */
void QDeviceStatus::updateDeviceColor(RoverCan2::Constant::eDeviceId deviceID_, const rover_msgs::msg::CanDeviceStatus& msg_)
{
    if (!_deviceInfo.contains(deviceID_) || !_deviceReboot.contains(deviceID_))
    {
        return;
    }

    QWidget* widgetInfo = _deviceInfo[deviceID_];

    switch (msg_.error_state)
    {
        case rover_msgs::msg::CanDeviceStatus::STATUS_OK:
            widgetInfo->setStyleSheet(STATUS_SUCCESS);
            break;
        case rover_msgs::msg::CanDeviceStatus::STATUS_WARNING:
            widgetInfo->setStyleSheet(STATUS_WARNING);
            break;
        case rover_msgs::msg::CanDeviceStatus::STATUS_ERROR:
            widgetInfo->setStyleSheet(STATUS_ERROR);
            break;
        default:
            widgetInfo->setStyleSheet(STATUS_DEFAULT);
            break;
    }
}

/**
 * @brief Sets the default style for all device labels.
 *
 */
void QDeviceStatus::setDefaultStyle()
{
    for (auto& it : _deviceInfo)
    {
        it.second->setStyleSheet(STATUS_DEFAULT);
    }
}

std::string QDeviceStatus::getDeviceName(RoverCan2::Constant::eDeviceId deviceID_)
{
    switch (deviceID_)
    {
        case RoverCan2::Constant::eDeviceId::FRONTLEFT_MOTOR:
            return "Front Left Motor";
        case RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR:
            return "Front Right Motor";
        case RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR:
            return "Rear Left Motor";
        case RoverCan2::Constant::eDeviceId::REARRIGHT_MOTOR:
            return "Rear Right Motor";
        case RoverCan2::Constant::eDeviceId::GNSS:
            return "GNSS";
        case RoverCan2::Constant::eDeviceId::DDB_CONTROLLER:
            return "DDB Controller";
        case RoverCan2::Constant::eDeviceId::CAMERA_ROVER_MAIN:
            return "Camera Rover Main";
        case RoverCan2::Constant::eDeviceId::CAMERA_ROVER_ANTENNA:
            return "Camera Rover Antenna";
        case RoverCan2::Constant::eDeviceId::LIGHTS_MAIN:
            return "Lights Main";
        default:
            return "Unknown Device";
    }
}