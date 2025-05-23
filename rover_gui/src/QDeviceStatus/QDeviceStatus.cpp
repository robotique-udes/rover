#include "QDeviceStatus.hpp"
#include <QStyle>

constexpr const char* STATUS_DEFAULT = "QLabel {"
                                       "background-color: #3c3f41;"
                                       "border: 1px solid #4b4e52;"
                                       "border-radius: 5px;"
                                       "padding: 5px 10px;"
                                       "}";

constexpr const char* STATUS_SUCCESS = "QLabel {"
                                       "background-color: #81c784;"
                                       "color: black;"
                                       "border: 1px solid #388e3c;"
                                       "border-radius: 5px;"
                                       "padding: 5px 10px;"
                                       "}";

constexpr const char* STATUS_WARNING = "QLabel {"
                                       "background-color : #ffb74d;"
                                       "color: black;"
                                       "border: 1px solid #e65100;"
                                       "border-radius : 5px;"
                                       "padding: 5px 10px;"
                                       "}";

constexpr const char* STATUS_ERROR = "QLabel {"
                                     "background-color : #e57373;"
                                     "color: black;"
                                     "border: 1px solid #b71c1c;"
                                     "border-radius : 5px;"
                                     "padding: 5px 10px;"
                                     "}";

QDeviceStatus::QDeviceStatus(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    _ui.setupUi(this);
    _QStatusWorker = new QStatusWorker(true, this);

    _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTLEFT_MOTOR)] = _ui.frontleftMotorInfo;
    _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR)] = _ui.frontrightMotorInfo;
    _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR)] = _ui.rearleftMotorInfo;
    _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARRIGHT_MOTOR)] = _ui.rearrightMotorInfo;
    _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::GNSS)] = _ui.gnssInfo;
    _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::DDB_CONTROLLER)] = _ui.ddbControllerInfo;
    _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::CAMERA_ROVER_MAIN)] = _ui.cameraMainInfo;
    _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::CAMERA_ROVER_ANTENNA)] = _ui.cameraAntenneInfo;
    _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::LIGHTS_MAIN)] = _ui.lightsMainInfo;
    // _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::SWITCHETH0)] = _ui.switchETH0Info;
    // _deviceInfo[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::SWITCHETH1)] = _ui.switchETH1Info;

    _deviceReboot[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTLEFT_MOTOR)] = _ui.frontleftMotorReboot;
    _deviceReboot[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR)] = _ui.frontrightMotorReboot;
    _deviceReboot[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR)] = _ui.rearleftMotorReboot;
    _deviceReboot[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARRIGHT_MOTOR)] = _ui.rearrightMotorReboot;
    _deviceReboot[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::GNSS)] = _ui.gnssReboot;
    _deviceReboot[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::DDB_CONTROLLER)] = _ui.ddbControllerReboot;
    _deviceReboot[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::CAMERA_ROVER_MAIN)] = _ui.cameraMainReboot;
    _deviceReboot[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::CAMERA_ROVER_ANTENNA)] = _ui.cameraAntenneReboot;
    _deviceReboot[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::LIGHTS_MAIN)] = _ui.lightsMainReboot;
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

    connect(_QStatusWorker,
            &QStatusWorker::onRequestDeviceStatusSuccessful,
            this,
            [this](bool success, const std::string response)
            {
                this->onRequestDeviceStatusSuccessful(success, response);
            });
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

    std::string deviceName = RoverCan2::Constant::getCanDeviceName(static_cast<RoverCan2::Constant::eDeviceId>(deviceID_));

    QString labelText = "Number of reboots: " + QString::number(_numberOfDeviceReboots[deviceID_]);

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

    auto labelInfo = _deviceInfo[deviceID_];
    auto labelReboot = _deviceReboot[deviceID_];

    switch (deviceStatus_.error_state)
    {
        case rover_msgs::msg::CanDeviceStatus::STATUS_OK:
            labelInfo->setStyleSheet(STATUS_SUCCESS);
            labelReboot->setStyleSheet(STATUS_SUCCESS);
            break;
        case rover_msgs::msg::CanDeviceStatus::STATUS_WARNING:
            labelInfo->setStyleSheet(STATUS_WARNING);
            labelReboot->setStyleSheet(STATUS_WARNING);
            break;
        case rover_msgs::msg::CanDeviceStatus::STATUS_ERROR:
            labelInfo->setStyleSheet(STATUS_ERROR);
            labelReboot->setStyleSheet(STATUS_ERROR);
            break;
        default:
            labelInfo->setStyleSheet(STATUS_DEFAULT);
            labelReboot->setStyleSheet(STATUS_DEFAULT);
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
        _deviceReboot[it.key()]->setStyleSheet(STATUS_DEFAULT);
        it.value()->setStyleSheet(STATUS_DEFAULT);
    }
}