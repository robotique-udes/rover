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
    _deviceButtons[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTLEFT_MOTOR)] = _ui.frontleftMotor;
    _deviceButtons[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR)] = _ui.rearleftMotor;
    _deviceButtons[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARRIGHT_MOTOR)] = _ui.rearrightMotor;
    _deviceButtons[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::DDB_CONTROLLER)] = _ui.ddbController;
    _deviceButtons[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::GNSS)] = _ui.gnss;

    _deviceLabels[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR)] = _ui.frontrightmotor_info;
    // _deviceLabels[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTLEFT_MOTOR)] = _ui.frontleftMotor;
    // _deviceLabels[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR)] = _ui.rearleftMotor;
    // _deviceLabels[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARRIGHT_MOTOR)] = _ui.rearrightMotor;
    // _deviceLabels[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::DDB_CONTROLLER)] = _ui.ddbController;
    // _deviceLabels[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::GNSS)] = _ui.gnss;

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
                auto request = std::make_shared<rover_msgs::srv::Empty::Request>();
                // this->setStatusReport(deviceID);
                this->updateDeviceInfo(request);
            });

    // Set the QSizePolicy to ensure aspect ratio resizing
    // QSizePolicy sp = this->sizePolicy();
    // sp.setHorizontalPolicy(QSizePolicy::Preferred);
    // sp.setVerticalPolicy(QSizePolicy::Preferred);
    // sp.setHeightForWidth(true);  // Enable height for width
    // this->setSizePolicy(sp);
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
                _numberOfCalls++;
                RCLCPP_INFO(_node->get_logger(), "Calls count: %d", _numberOfCalls);
                RCLCPP_WARN(_node->get_logger(), "Service failed: %s", response->message.c_str());
            }
        });
}

void QDeviceStatus::callbackDeviceInfos(const rover_msgs::msg::CanDeviceStatus& msg_)
{
    _deviceMessageCount[msg_.id]++;
    RCLCPP_INFO(_node->get_logger(), "Message count for device %d: %d", msg_.id, _deviceMessageCount[msg_.id]);

    if (isDeviceDisconnected(msg_.id))
    {
        _deviceLabels[msg_.id]->setStyleSheet(STATUS_DEFAULT);
    }
    else
    {
        this->updateDevicesColor(msg_.id, msg_);
    }
}

bool QDeviceStatus::isDeviceDisconnected(uint16_t deviceID_)
{
    _numberOfDeviceReboots[deviceID_] = _numberOfCalls - _deviceMessageCount[deviceID_];
    RCLCPP_INFO(_node->get_logger(), "Number of reboots for device %d: %d", deviceID_, _numberOfDeviceReboots[deviceID_]);

    if (_numberOfDeviceReboots[deviceID_] != _oldDeviceReboots[deviceID_])
    {
        _oldDeviceReboots[deviceID_] = _numberOfDeviceReboots[deviceID_];
        RCLCPP_INFO(_node->get_logger(), "Device %d is deconnected", deviceID_);
        return true;
    }
    else
    {
        RCLCPP_INFO(_node->get_logger(), "Device %d isn't deconnected", deviceID_);
        return false;
    }
}

void QDeviceStatus::rebootDevice(uint16_t id_)
{
    RCLCPP_INFO(_node->get_logger(), "Reboot %d", id_);
}

// int QDeviceStatus::heightForWidth(int width_) const
// {
//     // Load the image and get the aspect ratio
//     QPixmap pixmap(":/images/rover.png");  // Path to your image in resources
//     int originalWidth = pixmap.width();
//     int originalHeight = pixmap.height();

//     // Calculate height based on width, keeping the same aspect ratio
//     int height = width_ * originalHeight / originalWidth;
//     return height;
// }

void QDeviceStatus::setStatusReport(uint16_t deviceID_)
{
    auto label = _deviceLabels[deviceID_];

    std::string deviceName = this->getDeviceName(deviceID_);

    QString labelText = QString::fromStdString(deviceName) + "\n" + "0x" + QString::number(deviceID_, 16).toUpper() + "\n"
                        + "Nombre reboot: " + QString::number(_numberOfDeviceReboots[deviceID_]);

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

void QDeviceStatus::updateDevicesColor(uint16_t deviceID_, const rover_msgs::msg::CanDeviceStatus deviceStatus_)
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