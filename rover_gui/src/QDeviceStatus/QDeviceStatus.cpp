#include "QDeviceStatus.hpp"

QDeviceStatus::QDeviceStatus(std::shared_ptr<rclcpp::Node> guiNode_ , QWidget* parent_) : QWidget(parent_), _node(guiNode_)
{
	_ui.setupUi(this);
    connect(_ui.test, &QPushButton::clicked, this, &QDeviceStatus::setStatusReport(RoverCan2::Constant::eDeviceId::MASTER_COMPUTER_UNIT));

	_sub_deviceStatus = _node->create_subscription<rover_msgs::msg::CanDeviceStatus>(
		"/rover/can/devices_status",
		10,
		[this](const rover_msgs::msg::CanDeviceStatus::SharedPtr msg) {
			this->callbackDeviceInfos(*msg);
		}
	);


    // Set the QSizePolicy to ensure aspect ratio resizing
    QSizePolicy sp = this->sizePolicy();
    sp.setHorizontalPolicy(QSizePolicy::Preferred);
    sp.setVerticalPolicy(QSizePolicy::Preferred);
    sp.setHeightForWidth(true);  // Enable height for width
    this->setSizePolicy(sp);
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
}

void QDeviceStatus::setStatusReport(RoverCan2::Constant::eDeviceId id_)
{
    uint16_t deviceID = TO_UNDERLYING(id_);

    auto label = _ui.StatusReport->findChild<QLabel*>("StatusInfo");
    if (!label) {
        RCLCPP_ERROR(_node->get_logger(), "StatusInfo not found!");
        return;
    }


    // Look up device info
    auto it = _deviceStatusInfo.find(deviceID);
    if (it != _deviceStatusInfo.end()) {
        const auto& deviceStatus = it->second;

        // Compose your status string however you like
        QString statusText = QString("ID: %1\nStatus: %2")
                             .arg(deviceStatus.id)
                             .arg(this->setErrorMsg(deviceStatus.error_state));

        // Set label text
        label->setText(statusText);
    } else {
        label->setText("Device not found.");
    }
}

QString QDeviceStatus::setErrorMsg(uint8_t errorCode_)
{
    switch (errorCode_)
    {
        case rover_msgs::msg::CanDeviceStatus::STATUS_OK:
            return "OK";
            break;

        case rover_msgs::msg::CanDeviceStatus::STATUS_WARNING:
            return "Warning";
            break;

        case rover_msgs::msg::CanDeviceStatus::STATUS_ERROR:
            return "Error";
            break;

        default:
            return "Unknown";
            break;
    }
}