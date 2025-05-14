#include "QDeviceStatus.hpp"
#include <QStyle>

QDeviceStatus::QDeviceStatus(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    _ui.setupUi(this);

    _deviceButtons[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR)] = _ui.frontrightMotor;
    _deviceButtons[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTLEFT_MOTOR)] = _ui.frontleftMotor;
    _deviceButtons[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR)] = _ui.rearleftMotor;
    _deviceButtons[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR)] = _ui.rearrightMotor;
    _deviceButtons[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::DDB_CONTROLLER)] = _ui.ddbController;
    _deviceButtons[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::GNSS)] = _ui.gnss;

    for (auto it = _deviceButtons.begin(); it != _deviceButtons.end(); ++it)
    {
        uint16_t deviceID = it.key(); 
        QPushButton* button = it.value();

        connect(button,
                &QPushButton::clicked,
                this,
                [this, deviceID]()
                {
                    this->setStatusReport(deviceID);
                });
    }

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
        this->setStatusReport(msg_.id);
    }

    auto buttonIt = _deviceButtons.find(msg_.id);
    if (buttonIt != _deviceButtons.end())
    {
        this->updateDeviceButtonColor(buttonIt.value(), msg_);
    }
}

void QDeviceStatus::setStatusReport(uint16_t id_)
{
    auto groupBoxLabel = _ui.StatusReport->findChild<QLabel*>("StatusInfo");
    if (!groupBoxLabel)
    {
        RCLCPP_ERROR(_node->get_logger(), "StatusInfo not found!");
        return;
    }

    // Look up device info
    auto it = _deviceStatusInfo.find(id_);
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
        _currentStatusID = id_;
        RCLCPP_ERROR(_node->get_logger(), "Device ID %d not found in device status info.", id_);
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
            return "Device is currently in error";
            break;

        default:
            return "Unknown";
            break;
    }
}

void QDeviceStatus::updateDeviceButtonColor(QPushButton* button_, const rover_msgs::msg::CanDeviceStatus& deviceStatus_)
{
    QString style_default = QString("QPushButton {"
                                    "background-color: #3c3f41;"
                                    "border: 1px solid #4b4e52;"
                                    "border-radius: 5px;"
                                    "padding: 5px 10px;"
                                    "}");

    if (!deviceStatus_.watchdog_ok)
    {
        button_->setStyleSheet(style_default);
        return;
    }

    QString className;
    switch (deviceStatus_.error_state)
    {
        case rover_msgs::msg::CanDeviceStatus::STATUS_OK:
            className = "success";
            break;
        case rover_msgs::msg::CanDeviceStatus::STATUS_WARNING:
            className = "warning";
            break;
        case rover_msgs::msg::CanDeviceStatus::STATUS_ERROR:
            className = "error";
            break;
        default:
            button_->setStyleSheet(style_default);
            return;
    }

    if (CHECK_POINTER_VALID(button_))
    {
        button_->setProperty("class", className);
        button_->style()->unpolish(button_);
        button_->style()->polish(button_);
    }
    else
    {
        assert(false && "Button pointer is not valid");
    }
}