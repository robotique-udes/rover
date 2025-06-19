#include "QDeviceStatus.hpp"

#include <rover_lib2/helpers/assert.hpp>
#include <QStyle>
#include <utility>

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
    ASSERT_COND(_node != nullptr);

    _ui.setupUi(this);

    _canDevices[RoverCan2::Constant::eDeviceId::FRONTLEFT_MOTOR] = {0, 0, _ui.frontleftMotor, _ui.frontleftMotorInfo};
    _canDevices[RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR] = {0, 0, _ui.frontrightMotor, _ui.frontrightMotorInfo};
    _canDevices[RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR] = {0, 0, _ui.rearleftMotor, _ui.rearleftMotorInfo};
    _canDevices[RoverCan2::Constant::eDeviceId::REARRIGHT_MOTOR] = {0, 0, _ui.rearrightMotor, _ui.rearrightMotorInfo};
    _canDevices[RoverCan2::Constant::eDeviceId::GNSS] = {0, 0, _ui.gnss, _ui.gnssInfo};
    _canDevices[RoverCan2::Constant::eDeviceId::DDB_CONTROLLER] = {0, 0, _ui.ddbController, _ui.ddbControllerInfo};
    _canDevices[RoverCan2::Constant::eDeviceId::CAMERA_ROVER_MAIN] = {0, 0, _ui.cameraMain, _ui.cameraMainInfo};
    _canDevices[RoverCan2::Constant::eDeviceId::CAMERA_ROVER_ANTENNA] = {0, 0, _ui.cameraAntenne, _ui.cameraAntenneInfo};
    _canDevices[RoverCan2::Constant::eDeviceId::LIGHTS_MAIN] = {0, 0, _ui.lightsMain, _ui.lightsMainInfo};
    // _canDevices[RoverCan2::Constant::eDeviceId::SWITCHETH0] = {0, 0, _ui.switchETH0, _ui.switchETH0Info};
    // _canDevices[RoverCan2::Constant::eDeviceId::SWITCHETH1] = {0, 0, _ui.switchETH1, _ui.switchETH1Info};

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
                this->resetWidget();
                this->updateDeviceInfo();
            });

    connect(&_QStatusWorker,
            &QStatusWorker::onRequestDeviceStatusSuccessful,
            this,
            &QDeviceStatus::onRequestDeviceStatusSuccessful);
}

void QDeviceStatus::hideControls()
{
    _ui.serviceCall->hide();
}

void QDeviceStatus::showControls()
{
    _ui.serviceCall->show();
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

    if (!_canDevices.contains(deviceID))
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Invalid device ID received: %d", msg_.id);
        return;
    }

    _canDevices[deviceID].deviceMessageCount++;

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
    uint16_t& deviceReboots = _canDevices[deviceID_].numberOfDeviceReboots;
    const uint16_t& deviceMessageCount = _canDevices[deviceID_].deviceMessageCount;

    if (deviceMessageCount > 1U)
    {
        deviceReboots = deviceMessageCount - 1;
        uint16_t deviceID = TO_UNDERLYING(deviceID_);
        RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Device 0x%X has rebooted since last call", deviceID);
        _canDevices[deviceID_].deviceInfo->setStyleSheet(STATUS_WARNING);
    }
}

/**
 * @brief Updates the label text with the updated status report for the received device.
 *
 * @param deviceID_
 */
void QDeviceStatus::setStatusReport(RoverCan2::Constant::eDeviceId deviceID_)
{
    QLabel* label = _canDevices[deviceID_].deviceReboot;

    std::string deviceName = this->getDeviceName(deviceID_);
    uint16_t deviceID = std::to_underlying(deviceID_);

    QString labelText = QString::fromStdString(deviceName) + QString("\n\nID: 0x%1").arg(deviceID, 3, 16, QChar('0'))
                        + "\n\nReboots: " + QString::number(_canDevices[deviceID_].numberOfDeviceReboots);

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
    QWidget* widgetInfo = _canDevices[deviceID_].deviceInfo;
    QString currentStyle = widgetInfo->styleSheet();

    switch (msg_.error_state)
    {
        case rover_msgs::msg::CanDeviceStatus::STATUS_OK:
            if (currentStyle != STATUS_WARNING)
            {
                widgetInfo->setStyleSheet(STATUS_SUCCESS);
            }
            break;
        case rover_msgs::msg::CanDeviceStatus::STATUS_WARNING:
            [[fallthrough]];
        case rover_msgs::msg::CanDeviceStatus::STATUS_ERROR:
            widgetInfo->setStyleSheet(STATUS_ERROR);
            break;
        default:
            widgetInfo->setStyleSheet(STATUS_DEFAULT);
            break;
    }
}

/**
 * @brief Resets the widget to its initial state.
 *
 */
void QDeviceStatus::resetWidget()
{
    for (auto& [key, value] : _canDevices)
    {
        if (value.deviceInfo)
        {
            value.deviceMessageCount = 0U;
            value.numberOfDeviceReboots = 0U;
            value.deviceInfo->setStyleSheet(STATUS_DEFAULT);
            this->setStatusReport(key);
        }
    }
}

std::string QDeviceStatus::getDeviceName(RoverCan2::Constant::eDeviceId deviceID_) const
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
