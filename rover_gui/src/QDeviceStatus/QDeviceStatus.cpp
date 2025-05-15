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

    // _deviceLabels[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR)] = _ui.frontrightMotor;
    // _deviceLabels[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTLEFT_MOTOR)] = _ui.frontleftMotor;
    // _deviceLabels[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR)] = _ui.rearleftMotor;
    // _deviceLabels[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::REARRIGHT_MOTOR)] = _ui.rearrightMotor;
    // _deviceLabels[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::DDB_CONTROLLER)] = _ui.ddbController;
    // _deviceLabels[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::GNSS)] = _ui.gnss;
    _deviceLabels[TO_UNDERLYING(RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR)] = _ui.frontrightmotor_info;

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
                RCLCPP_INFO(_node->get_logger(), "Service succeeded: %s", response->message.c_str());
            }
            else
            {
                RCLCPP_WARN(_node->get_logger(), "Service failed: %s", response->message.c_str());
            }
        });
}

void QDeviceStatus::callbackDeviceInfos(const rover_msgs::msg::CanDeviceStatus& msg_)
{
    _deviceStatusInfo[msg_.id] = msg_;

    this->updateDevicesColor();
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

void QDeviceStatus::setStatusReport(uint16_t id_) {}

void QDeviceStatus::updateDevicesColor()
{
    for (auto it = _deviceLabels.begin(); it != _deviceLabels.end(); ++it)
    {
        QLabel* label = it.value();
        uint16_t deviceID = it.key();

        auto statusIt = _deviceStatusInfo.find(deviceID);
        if (statusIt == _deviceStatusInfo.end())
        {
            // No status info for this device, set default color
            label->setStyleSheet(STATUS_DEFAULT);
            continue;
        }

        const auto& deviceStatus = statusIt->second;

        switch (deviceStatus.error_state)
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
    }
}