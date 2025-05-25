#include "QArbitration.hpp"
#include "Global/Helpers/QToastNotification/QToastNotification.hpp"
#include <cstdint>
#include <rover_msgs/msg/detail/drivetrain_arbitration__struct.hpp>
#include <rover_msgs/srv/detail/drive_train_arbitration__struct.hpp>

QArbitration::QArbitration(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    _ui.setupUi(this);

    this->initComboBoxItems();

    _clientJoy = _node->create_client<rover_msgs::srv::JoyDemuxSetState>(TOPIC_JOY_DEMUX_CONTROL);
    _clientDriveTrain = _node->create_client<rover_msgs::srv::DriveTrainArbitration>(TOPIC_DT_DEMUX_CONTROL);

    _joyDemuxStatusSub = _node->create_subscription<rover_msgs::msg::JoyDemuxStatus>(
        TOPIC_JOY_DEMUX_STATUS,
        10,
        std::bind(&QArbitration::JoyDemuxStatusCallback, this, std::placeholders::_1));

    _driveTrainStatusSub = _node->create_subscription<rover_msgs::msg::DrivetrainArbitration>(
        TOPIC_DT_DEMUX_STATUS,
        10,
        std::bind(&QArbitration::DriveTrainDemuxStatusCallback, this, std::placeholders::_1));

    connect(_ui.mainComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &QArbitration::onMainComboChanged);
    connect(_ui.secComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &QArbitration::onSecComboChanged);
    connect(_ui.driveTrainComboBox,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            &QArbitration::onDriveTrainComboChanged);
}

void QArbitration::initComboBoxItems()
{
    this->_ui.mainComboBox->addItem("Drive Train", 0);
    this->_ui.mainComboBox->addItem("Arm", 1);
    this->_ui.mainComboBox->addItem("Antenna", 2);
    this->_ui.mainComboBox->addItem("None", 3);

    this->_ui.secComboBox->addItem("Drive Train", 0);
    this->_ui.secComboBox->addItem("Arm", 1);
    this->_ui.secComboBox->addItem("Antenna", 2);
    this->_ui.secComboBox->addItem("None", 3);

    this->_ui.driveTrainComboBox->addItem("None", 0);
    this->_ui.driveTrainComboBox->addItem("Teleop", 1);
    this->_ui.driveTrainComboBox->addItem("Autonomous", 2);
}

void QArbitration::onMainComboChanged(int index)
{
    this->checkServiceAvailable<rover_msgs::srv::JoyDemuxSetState>(_clientJoy, TOPIC_JOY_DEMUX_CONTROL);

    auto request = std::make_shared<rover_msgs::srv::JoyDemuxSetState::Request>();
    request->controller_type = static_cast<int8_t>(eControllerType::main);
    request->destination = index;
    request->force = true;

    RCLCPP_WARN(_node->get_logger(), "Sending Main request");
    auto result = _clientJoy->async_send_request(request);
}

void QArbitration::onSecComboChanged(int index)
{
    this->checkServiceAvailable<rover_msgs::srv::JoyDemuxSetState>(_clientJoy, TOPIC_JOY_DEMUX_CONTROL);

    auto request = std::make_shared<rover_msgs::srv::JoyDemuxSetState::Request>();
    request->controller_type = static_cast<int8_t>(eControllerType::secondary);
    request->destination = index;
    request->force = false;

    auto result = _clientJoy->async_send_request(request);
}

void QArbitration::onDriveTrainComboChanged(int index)
{
    this->checkServiceAvailable<rover_msgs::srv::DriveTrainArbitration>(_clientDriveTrain, TOPIC_DT_DEMUX_CONTROL);

    auto request = std::make_shared<rover_msgs::srv::DriveTrainArbitration::Request>();
    request->target_arbitration.arbitration = index;

    auto result = _clientDriveTrain->async_send_request(request);
}

template<typename T>
void QArbitration::checkServiceAvailable(rclcpp::Client<T>::SharedPtr client, const std::string& serviceName)
{
    if (!client->service_is_ready())
    {
        QHelper::QToastNotification::getInstance().notifyFromAnyThread("Service unavailable",
                                                                       "Couldn't send a request to " + serviceName
                                                                           + ", the service is unavailable.",
                                                                       QHelper::QToastNotification::eNotifType::WARNING,
                                                                       2'000);
    }
}

void QArbitration::JoyDemuxStatusCallback(const rover_msgs::msg::JoyDemuxStatus::SharedPtr msg)
{
    if (!msg)
    {
        RCLCPP_WARN(_node->get_logger(), "Received null JoyDemuxStatus message.");
        return;
    }

    bool wasBlockedMain = _ui.mainComboBox->blockSignals(true);
    bool wasBlockedSec = _ui.secComboBox->blockSignals(true);

    _ui.mainComboBox->setCurrentIndex(msg->controller_main_topic);
    _ui.secComboBox->setCurrentIndex(msg->controller_secondary_topic);

    _ui.mainComboBox->blockSignals(wasBlockedMain);
    _ui.secComboBox->blockSignals(wasBlockedSec);
}

void QArbitration::DriveTrainDemuxStatusCallback(const rover_msgs::msg::DrivetrainArbitration::SharedPtr msg)
{
    if (!msg)
    {
        RCLCPP_WARN(_node->get_logger(), "Received null drivetrain demux status message.");
        return;
    }

    bool wasBlockedMain = _ui.driveTrainComboBox->blockSignals(true);

    _ui.driveTrainComboBox->setCurrentIndex(msg->arbitration);

    _ui.driveTrainComboBox->blockSignals(wasBlockedMain);
}