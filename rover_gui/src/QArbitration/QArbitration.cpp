#include "QArbitration.hpp"
#include "Global/Helpers/QToastNotification/QToastNotification.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include <cstdint>
#include <limits>
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
        QOS_DEFAULT,
        [this](const rover_msgs::msg::JoyDemuxStatus::SharedPtr msg_)
        {
            emit this->joyDemuxStatusChanged(msg_);
        });

    _driveTrainStatusSub = _node->create_subscription<rover_msgs::msg::DrivetrainArbitration>(
        TOPIC_DT_DEMUX_STATUS,
        QOS_DEFAULT,
        [this](const rover_msgs::msg::DrivetrainArbitration::SharedPtr msg_)
        {
            emit this->driveTrainDemuxStatusChanged(msg_);
        });

    connect(_ui.mainComboBox,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            [this](int comboBoxIndex_)
            {
                this->onControllerComboChanged(eControllerType::MAIN, comboBoxIndex_);
            });
    connect(_ui.secComboBox,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            [this](int comboBoxIndex_)
            {
                this->onControllerComboChanged(eControllerType::SECONDARY, comboBoxIndex_);
            });

    connect(_ui.driveTrainComboBox,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            &QArbitration::onDriveTrainComboChanged);

    connect(this, &QArbitration::joyDemuxStatusChanged, this, &QArbitration::onJoyDemuxStatusChanged);

    connect(this, &QArbitration::driveTrainDemuxStatusChanged, this, &QArbitration::onDriveTrainDemuxStatusChanged);
}

void QArbitration::initComboBoxItems()
{
    this->_ui.mainComboBox->addItem("Drive Train", rover_msgs::srv::JoyDemuxSetState::Request::DEST_DRIVE_TRAIN);
    this->_ui.mainComboBox->addItem("Arm", rover_msgs::srv::JoyDemuxSetState::Request::DEST_ARM);
    this->_ui.mainComboBox->addItem("Antenna", rover_msgs::srv::JoyDemuxSetState::Request::DEST_ANTENNA);
    this->_ui.mainComboBox->addItem("Science", rover_msgs::srv::JoyDemuxSetState::Request::DEST_SCIENCE);
    this->_ui.mainComboBox->addItem("None", rover_msgs::srv::JoyDemuxSetState::Request::DEST_NONE);

    this->_ui.secComboBox->addItem("Drive Train", rover_msgs::srv::JoyDemuxSetState::Request::DEST_DRIVE_TRAIN);
    this->_ui.secComboBox->addItem("Arm", rover_msgs::srv::JoyDemuxSetState::Request::DEST_ARM);
    this->_ui.secComboBox->addItem("Antenna", rover_msgs::srv::JoyDemuxSetState::Request::DEST_ANTENNA);
    this->_ui.secComboBox->addItem("Science", rover_msgs::srv::JoyDemuxSetState::Request::DEST_SCIENCE);
    this->_ui.secComboBox->addItem("None", rover_msgs::srv::JoyDemuxSetState::Request::DEST_NONE);

    this->_ui.driveTrainComboBox->addItem("None", 0);
    this->_ui.driveTrainComboBox->addItem("Teleop", 1);
    this->_ui.driveTrainComboBox->addItem("Autonomous", 2);
}

void QArbitration::onControllerComboChanged(eControllerType controller_, int index_)
{
    uint8_t indexInt = 0U;
    if (index_ < 0 && index_ > std::numeric_limits<uint8_t>::max())
    {
        RCLCPP_WARN(_node->get_logger(), "Implementation error, combobox index (%u) not a uint8_t value as expected", index_);
        return;
    }
    indexInt = static_cast<uint8_t>(index_);

    this->checkServiceAvailable<rover_msgs::srv::JoyDemuxSetState>(_clientJoy, TOPIC_JOY_DEMUX_CONTROL);

    auto request = std::make_shared<rover_msgs::srv::JoyDemuxSetState::Request>();
    request->controller_type = TO_UNDERLYING(controller_);
    request->destination = indexInt;
    request->force = true;

    _clientJoy->async_send_request(request);
}

void QArbitration::onDriveTrainComboChanged(int index_)
{
    uint8_t indexInt = 0U;
    if (index_ < 0 && index_ > std::numeric_limits<uint8_t>::max())
    {
        RCLCPP_WARN(_node->get_logger(), "Implementation error, combobox index (%u) not a uint8_t value as expected", index_);
        return;
    }
    indexInt = static_cast<uint8_t>(index_);

    this->checkServiceAvailable<rover_msgs::srv::DriveTrainArbitration>(_clientDriveTrain, TOPIC_DT_DEMUX_CONTROL);

    auto request = std::make_shared<rover_msgs::srv::DriveTrainArbitration::Request>();
    request->target_arbitration.arbitration = indexInt;

    _clientDriveTrain->async_send_request(request);
}

template<typename T>
void QArbitration::checkServiceAvailable(rclcpp::Client<T>::SharedPtr client_, const std::string& serviceName_)
{
    if (!client_->service_is_ready())
    {
        QHelper::QToastNotification::getInstance().notifyFromAnyThread("Service unavailable",
                                                                       "Couldn't send a request to " + serviceName_
                                                                           + ", the service is unavailable.",
                                                                       QHelper::QToastNotification::eNotifType::WARNING,
                                                                       2'000);
    }
}

void QArbitration::onJoyDemuxStatusChanged(const rover_msgs::msg::JoyDemuxStatus::SharedPtr msg_)
{
    if (!msg_)
    {
        RCLCPP_WARN(_node->get_logger(), "Received null JoyDemuxStatus message.");
        return;
    }

    bool wasBlockedMain = _ui.mainComboBox->blockSignals(true);
    bool wasBlockedSec = _ui.secComboBox->blockSignals(true);

    _ui.mainComboBox->setCurrentIndex(msg_->controller_main_topic);
    _ui.secComboBox->setCurrentIndex(msg_->controller_secondary_topic);

    _ui.mainComboBox->blockSignals(wasBlockedMain);
    _ui.secComboBox->blockSignals(wasBlockedSec);
}

void QArbitration::onDriveTrainDemuxStatusChanged(const rover_msgs::msg::DrivetrainArbitration::SharedPtr msg)
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
