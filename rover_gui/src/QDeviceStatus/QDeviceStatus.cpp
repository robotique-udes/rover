#include "QDeviceStatus.hpp"

QDeviceStatus::QDeviceStatus(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    _ui.setupUi(this);
    
    connect(_ui.test,
            &QPushButton::clicked,
            this,
            [this]()
            {
                this->setStatusReport(RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR);
            });

    connect(_ui.test_2,
            &QPushButton::clicked,
            this,
            [this]()
            {
                this->setStatusReport(RoverCan2::Constant::eDeviceId::FRONTLEFT_MOTOR);
            });

    _sub_deviceStatus = _node->create_subscription<rover_msgs::msg::CanDeviceStatus>(
        "/rover/can/devices_status",
        10,
        [this](const rover_msgs::msg::CanDeviceStatus::SharedPtr msg)
        {
            this->callbackDeviceInfos(*msg);
        });

    // Set the QSizePolicy to ensure aspect ratio resizing
    // QSizePolicy sp = this->sizePolicy();
    // sp.setHorizontalPolicy(QSizePolicy::Preferred);
    // sp.setVerticalPolicy(QSizePolicy::Preferred);
    // sp.setHeightForWidth(true);  // Enable height for width
    // this->setSizePolicy(sp);
}

int QDeviceStatus::heightForWidth(int width_) const
{
    // Load the image and get the aspect ratio
    QPixmap pixmap(":/images/rover.png");  // Path to your image in resources
    int originalWidth = pixmap.width();
    int originalHeight = pixmap.height();

    // Calculate height based on width, keeping the same aspect ratio
    int height = width_ * originalHeight / originalWidth;
    return height;
}

void QDeviceStatus::callbackDeviceInfos(const rover_msgs::msg::CanDeviceStatus& msg_)
{
    _deviceStatusInfo[msg_.id] = msg_;

    if (_currentStatusID == msg_.id)
    {
        this->setStatusReport(static_cast<RoverCan2::Constant::eDeviceId>(msg_.id));
    }
}

void QDeviceStatus::setStatusReport(RoverCan2::Constant::eDeviceId id_)
{
    uint16_t deviceID = TO_UNDERLYING(id_);

    auto groupBoxLabel = _ui.StatusReport->findChild<QLabel*>("StatusInfo");
    if (!groupBoxLabel)
    {
        RCLCPP_ERROR(_node->get_logger(), "StatusInfo not found!");
        return;
    }

    // Look up device info
    auto it = _deviceStatusInfo.find(deviceID);
    if (it != _deviceStatusInfo.end())
    {
        const auto& deviceStatus = it->second;

        groupBoxLabel->setTextFormat(Qt::RichText);
        // Set QString to same as UI_DeviceStatus.h
        QString statusText = QString("<html><head/><body>"
                                     "<p>Device ID: %1</p>"
                                     "<p>Status: %2</p>"
                                     "<p>Watchdog: %3</p>"
                                     "</body></html>")
                                 .arg(deviceStatus.id)
                                 .arg(this->setErrorMsg(deviceStatus.error_state))
                                 .arg(deviceStatus.watchdog_ok ? "Still active" : "Not active");

        // Set label text
        groupBoxLabel->setText(statusText);
        _currentStatusID = deviceStatus.id;
    }
    else
    {
        groupBoxLabel->setText("Device not found.");
        _currentStatusID = deviceID;
        RCLCPP_ERROR(_node->get_logger(), "Device ID %d not found in device status info.", deviceID);
    }
}

QString QDeviceStatus::setErrorMsg(uint8_t errorCode_)
{
    switch (errorCode_)
    {
        case rover_msgs::msg::CanDeviceStatus::STATUS_OK:
            return "Device is currently OK";
            break;

        case rover_msgs::msg::CanDeviceStatus::STATUS_WARNING:
            return "Device is currently in warning";
            break;

        case rover_msgs::msg::CanDeviceStatus::STATUS_ERROR:
            return "Device is currently in Error";
            break;

        default:
            return "Unknown";
            break;
    }
}