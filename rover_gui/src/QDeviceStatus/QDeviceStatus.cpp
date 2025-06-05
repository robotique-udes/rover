#include "QDeviceStatus.hpp"
#include <QStyle>

constexpr const char* STATUS_DEFAULT = "QLabel {"
                                       "background-color: #3c3f41;"
                                       "border: 1px solid #4b4e52;"
                                       "}";

constexpr const char* STATUS_SUCCESS = "QWidget {"
                                       "background-color: #81c784;"
                                       "color: black;"
                                       "border: 1px solid #388e3c;"
                                       "}";

constexpr const char* STATUS_WARNING = "QLabel {"
                                       "background-color : #ffb74d;"
                                       "color: black;"
                                       "border: 1px solid #e65100;"
                                       "}";

constexpr const char* STATUS_ERROR = "QLabel {"
                                     "background-color : #e57373;"
                                     "color: black;"
                                     "border: 1px solid #b71c1c;"
                                     "}";

QDeviceStatus::QDeviceStatus(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    _ui.setupUi(this);

    _QStatusWorker = std::make_shared<QStatusWorker>(true, this);

    _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTLEFT_MOTOR)] = _ui.frontleftMotor;
    _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR)] = _ui.frontrightMotor;
    _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR)] = _ui.rearleftMotor;
    _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARRIGHT_MOTOR)] = _ui.rearrightMotor;
    _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::GNSS)] = _ui.gnss;
    _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::DDB_CONTROLLER)] = _ui.ddbController;
    _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::CAMERA_ROVER_MAIN)] = _ui.cameraMain;
    _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::CAMERA_ROVER_ANTENNA)] = _ui.cameraAntenne;
    _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::LIGHTS_MAIN)] = _ui.lightsMain;
    // _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::SWITCHETH0)] = _ui.switchETH0;
    // _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::SWITCHETH1)] = _ui.switchETH1;

    _deviceReboot[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTLEFT_MOTOR)] = _ui.frontleftMotorInfo;
    _deviceReboot[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR)] = _ui.frontrightMotorInfo;
    _deviceReboot[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR)] = _ui.rearleftMotorInfo;
    _deviceReboot[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARRIGHT_MOTOR)] = _ui.rearrightMotorInfo;
    _deviceReboot[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::GNSS)] = _ui.gnssInfo;
    _deviceReboot[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::DDB_CONTROLLER)] = _ui.ddbControllerInfo;
    _deviceReboot[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::CAMERA_ROVER_MAIN)] = _ui.cameraMainInfo;
    _deviceReboot[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::CAMERA_ROVER_ANTENNA)] = _ui.cameraAntenneInfo;
    _deviceReboot[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::LIGHTS_MAIN)] = _ui.lightsMainInfo;
    // _deviceReboot[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::SWITCHETH0)] = _ui.switchETH0Reboot;
    // _deviceReboot[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::SWITCHETH1)] = _ui.switchETH1Reboot;

    _sub_deviceStatus = _node->create_subscription<rover_msgs::msg::CanDeviceStatus>(
        "/rover/can/devices_status",
        QOS_DEFAULT,
        [this](const rover_msgs::msg::CanDeviceStatus::SharedPtr msg)
        {
            QMetaObject::invokeMethod(
                this,
                [this, msg]()
                {
                    this->callbackDeviceInfos(*msg);
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

    connect(_QStatusWorker.get(),
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
    _QStatusWorker->requestDeviceStatusManager(_client_requestErrorStatus);
}

/**
 * @brief Handles the response from the service call
 *
 * @param success_
 * @param response_
 */
void QDeviceStatus::onRequestDeviceStatusSuccessful(bool success_, const std::string response_)
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
    _deviceMessageCount[msg_.id]++;

    this->updateRebootCounter(msg_.id);
    this->updateDeviceColor(msg_.id, msg_);
    this->setStatusReport(msg_.id);
}

/**
 * @brief Keeps track of the number of reboots for each device. Logs a message when a device reboots.
 *
 * @param deviceID_
 */
void QDeviceStatus::updateRebootCounter(uint16_t deviceID_)
{
    if (!_deviceInfo.contains(deviceID_) || !_deviceReboot.contains(deviceID_))
    {
        return;
    }

    _numberOfDeviceReboots[deviceID_]
        = _numberOfDeviceRebootsFromButton[deviceID_] + _numberOfCalls - _deviceMessageCount[deviceID_];

    if (_numberOfDeviceReboots[deviceID_] != _oldDeviceReboots[deviceID_])
    {
        _oldDeviceReboots[deviceID_] = _numberOfDeviceReboots[deviceID_];
        RCLCPP_INFO(rclcpp::get_logger("GUI"), "Device %d has rebooted since last call", deviceID_);
    }
}

/**
 * @brief Updates the label text with the updated status report for the received device.
 *
 * @param deviceID_
 */
void QDeviceStatus::setStatusReport(uint16_t deviceID_)
{
    if (!_deviceInfo.contains(deviceID_) || !_deviceReboot.contains(deviceID_))
    {
        return;
    }

    auto label = _deviceReboot[deviceID_];

    std::string deviceName = this->getDeviceName(deviceID_);

    QString labelText = QString::fromStdString(deviceName) + QString("\n\nID: 0x%1").arg(deviceID_, 3, 16, QChar('0'))
                        + "\n\nReboots: " + QString::number(_numberOfDeviceReboots[deviceID_]);

    label->setText(labelText);
}

/**
 * @brief Updates the label color based on the device status. If ok: green, if warning: yellow, if error: red.
 *
 * @param deviceID_
 * @param deviceStatus_
 */
void QDeviceStatus::updateDeviceColor(uint16_t deviceID_, const rover_msgs::msg::CanDeviceStatus& deviceStatus_)
{
    if (!_deviceInfo.contains(deviceID_) || !_deviceReboot.contains(deviceID_))
    {
        return;
    }

    auto widgetInfo = _deviceInfo[deviceID_];

    switch (deviceStatus_.error_state)
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
    for (auto it = _deviceInfo.begin(); it != _deviceInfo.end(); ++it)
    {
        it.value()->setStyleSheet(STATUS_DEFAULT);
    }
}

const std::string QDeviceStatus::getDeviceName(uint16_t deviceID_)
{
    switch (deviceID_)
    {
        case TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTLEFT_MOTOR):
            return "Front Left Motor";
        case TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR):
            return "Front Right Motor";
        case TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR):
            return "Rear Left Motor";
        case TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARRIGHT_MOTOR):
            return "Rear Right Motor";
        case TO_UNDERLYING(RoverCan2::Constant::eDeviceId::GNSS):
            return "GNSS";
        case TO_UNDERLYING(RoverCan2::Constant::eDeviceId::DDB_CONTROLLER):
            return "DDB Controller";
        case TO_UNDERLYING(RoverCan2::Constant::eDeviceId::CAMERA_ROVER_MAIN):
            return "Camera Rover Main";
        case TO_UNDERLYING(RoverCan2::Constant::eDeviceId::CAMERA_ROVER_ANTENNA):
            return "Camera Rover Antenna";
        case TO_UNDERLYING(RoverCan2::Constant::eDeviceId::LIGHTS_MAIN):
            return "Lights Main";
        default:
            return "Unknown Device";
    }
}