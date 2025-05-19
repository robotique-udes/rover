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

    _deviceButtons[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR)] = _ui.frontrightMotor_reboot;
    _deviceButtons[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTLEFT_MOTOR)] = _ui.frontleftMotor_reboot;
    _deviceButtons[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR)] = _ui.rearleftMotor_reboot;
    _deviceButtons[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARRIGHT_MOTOR)] = _ui.rearrightMotor_reboot;
    _deviceButtons[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::DDB_CONTROLLER)] = _ui.ddbController_reboot;
    _deviceButtons[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::GNSS)] = _ui.gnss_reboot;

    _deviceLabels[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR)] = _ui.frontrightmotor_info;
    _deviceLabels[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTLEFT_MOTOR)] = _ui.frontleftmotor_info;
    _deviceLabels[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR)] = _ui.rearleftmotor_info;
    _deviceLabels[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARRIGHT_MOTOR)] = _ui.rearrightmotor_info;
    _deviceLabels[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::DDB_CONTROLLER)] = _ui.ddbController_info;
    _deviceLabels[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::GNSS)] = _ui.gnss_info;

    _sub_deviceStatus = _node->create_subscription<rover_msgs::msg::CanDeviceStatus>(
        "/rover/can/devices_status",
        10,
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

    _client = _node->create_client<rover_msgs::srv::Empty>("/rover/can/request_error_state");
    auto request = std::make_shared<rover_msgs::srv::Empty::Request>();
    this->updateDeviceInfo(request);

    for (auto it = _deviceButtons.begin(); it != _deviceButtons.end(); ++it)
    {
        uint16_t deviceID = it.key();
        QPushButton* button = it.value();

        connect(button,
                &QPushButton::clicked,
                this,
                [this, deviceID]()
                {
                    // this->setStatusReport(deviceID);
                    this->rebootDevice(deviceID);
                });
    }

    connect(_ui.pb_serviceCall,
            &QPushButton::clicked,
            this,
            [this]()
            {
                this->setDefaultStyle();
                auto request = std::make_shared<rover_msgs::srv::Empty::Request>();
                this->updateDeviceInfo(request);
            });

    // Load and assign pixmap to image label
    _pixmap = QPixmap(":/rovUS/images/rover2.jpeg");
    _imageLabel = _ui.RoverImage;
    _imageLabel->setPixmap(_pixmap);
    _imageLabel->setScaledContents(true);  // optionally if you want to fill label always

    // Store initial relative overlay positions
    _overlayPositions[_ui.frontleftMotor] = QPointF(0.0172, 0.3372);
    _overlayPositions[_ui.frontrightMotor] = QPointF(0.6532, 0.3372);
    _overlayPositions[_ui.rearleftMotor] = QPointF(0.0172, 0.6086);
    _overlayPositions[_ui.rearrightMotor] = QPointF(0.6532, 0.6086);
    _overlayPositions[_ui.ddbController] = QPointF(0.3442, 0.5263);
    _overlayPositions[_ui.gnss] = QPointF(0.2926, 0.7017);
    _overlayPositions[_ui.pb_serviceCall] = QPointF(0.7556, 0.0270);
    // Add for each of your overlay widgets...
}

void QDeviceStatus::updateDeviceInfo(std::shared_ptr<rover_msgs::srv::Empty::Request> request_)
{
    auto result_future = _client->async_send_request(
        request_,
        [this](rclcpp::Client<rover_msgs::srv::Empty>::SharedFuture future)
        {
            auto response = future.get();
            if (response->success)
            {
                _numberOfCalls++;
                RCLCPP_INFO(_node->get_logger(), "Calls count: %d", _numberOfCalls);
                RCLCPP_INFO(_node->get_logger(), "Service succeeded: %s", response->message.c_str());
            }
            else
            {
                // Reminder to remove this once everything is done
                _numberOfCalls++;
                RCLCPP_INFO(_node->get_logger(), "Calls count: %d", _numberOfCalls);
                //
                RCLCPP_WARN(_node->get_logger(), "Service failed: %s", response->message.c_str());
            }
        });
}

void QDeviceStatus::callbackDeviceInfos(const rover_msgs::msg::CanDeviceStatus& msg_)
{
    _deviceMessageCount[msg_.id]++;
    RCLCPP_INFO(_node->get_logger(), "Message count for device %d: %d", msg_.id, _deviceMessageCount[msg_.id]);

    this->updateRebootCounter(msg_.id);
    this->updateDeviceColor(msg_.id, msg_);
    this->setStatusReport(msg_.id);
}

void QDeviceStatus::updateRebootCounter(uint16_t deviceID_)
{
    _numberOfDeviceReboots[deviceID_] = _numberOfCalls - _deviceMessageCount[deviceID_];
    RCLCPP_INFO(_node->get_logger(), "Number of reboots for device %d: %d", deviceID_, _numberOfDeviceReboots[deviceID_]);

    if (_numberOfDeviceReboots[deviceID_] != _oldDeviceReboots[deviceID_])
    {
        _oldDeviceReboots[deviceID_] = _numberOfDeviceReboots[deviceID_];
        RCLCPP_INFO(_node->get_logger(), "Device %d is deconnected", deviceID_);
    }
    else
    {
        RCLCPP_INFO(_node->get_logger(), "Device %d isn't deconnected", deviceID_);
    }
}

void QDeviceStatus::rebootDevice(uint16_t deviceID_)
{
    RCLCPP_INFO(_node->get_logger(), "Reboot %d", deviceID_);
    _numberOfDeviceReboots[deviceID_]++;
}

void QDeviceStatus::setStatusReport(uint16_t deviceID_)
{
    auto label = _deviceLabels[deviceID_];

    std::string deviceName = this->getDeviceName(deviceID_);

    QString labelText = QString::fromStdString(deviceName) + "\n" + "0x" + QString::number(deviceID_, 16).toUpper() + "\n"
                        + "Number of reboots: " + QString::number(_numberOfDeviceReboots[deviceID_]);

    label->setText(labelText);
    RCLCPP_INFO(_node->get_logger(), "Label %d text changed", deviceID_);
}

std::string QDeviceStatus::getDeviceName(uint16_t deviceID_)
{
    switch (deviceID_)
    {
        case TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR):
            return "Front Right Motor";
        case TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTLEFT_MOTOR):
            return "Front Left Motor";
        case TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR):
            return "Rear Left Motor";
        case TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARRIGHT_MOTOR):
            return "Rear Right Motor";
        case TO_UNDERLYING(RoverCan2::Constant::eDeviceId::DDB_CONTROLLER):
            return "DDB Controller";
        case TO_UNDERLYING(RoverCan2::Constant::eDeviceId::GNSS):
            return "GNSS";
        default:
            return "Unknown Device";
    }
}

void QDeviceStatus::updateDeviceColor(uint16_t deviceID_, const rover_msgs::msg::CanDeviceStatus& deviceStatus_)
{
    auto label = _deviceLabels[deviceID_];

    switch (deviceStatus_.error_state)
    {
        case rover_msgs::msg::CanDeviceStatus::STATUS_OK:
            label->setStyleSheet(STATUS_SUCCESS);
            break;
        case rover_msgs::msg::CanDeviceStatus::STATUS_WARNING:
            label->setStyleSheet(STATUS_WARNING);
            break;
        case rover_msgs::msg::CanDeviceStatus::STATUS_ERROR:
            label->setStyleSheet(STATUS_ERROR);
            break;
        default:
            label->setStyleSheet(STATUS_DEFAULT);
            break;
    }

    RCLCPP_INFO(_node->get_logger(), "Label %d color changed", deviceID_);
}

void QDeviceStatus::setDefaultStyle()
{
    for (auto it = _deviceLabels.begin(); it != _deviceLabels.end(); ++it)
    {
        it.value()->setStyleSheet(STATUS_DEFAULT);
    }
}

void QDeviceStatus::resizeEvent(QResizeEvent* event_)
{
    QWidget::resizeEvent(event_);

    // Resize image label to fit widget while preserving aspect ratio
    QSize scaledSize = _pixmap.size();
    scaledSize.scale(size(), Qt::KeepAspectRatio);
    _imageLabel->resize(scaledSize);
    _imageLabel->move((width() - scaledSize.width()) / 2, (height() - scaledSize.height()) / 2);

    // Update overlays' absolute positions
    this->updateOverlayPositions();
}

void QDeviceStatus::updateOverlayPositions()
{
    QRect imageRect = _imageLabel->geometry();

    for (auto it = _overlayPositions.begin(); it != _overlayPositions.end(); ++it)
    {
        QWidget* overlay = it.key();
        QPointF relPos = it.value();

        int x = imageRect.left() + relPos.x() * imageRect.width();
        int y = imageRect.top() + relPos.y() * imageRect.height();

        overlay->move(x, y);
    }
}