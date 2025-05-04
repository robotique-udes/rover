#include "QDeviceStatus.hpp"

QDeviceStatus::QDeviceStatus(std::shared_ptr<rclcpp::Node> guiNode_ , QWidget* parent_) : QWidget(parent_), _node(guiNode_)
{
	_ui.setupUi(this);

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