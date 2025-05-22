#include "QArbitration.hpp"
#include "rover_lib2/helpers/log.hpp"
#include <rover_msgs/msg/detail/joy_demux_status__struct.hpp>

QArbitration::QArbitration(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    _ui.setupUi(this);

    this->initComboBoxItems();

    this->_clientJoy = _node->create_client<rover_msgs::srv::JoyDemuxSetState>("/base/joy/demux_control");

    connect(_ui.mainComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &QArbitration::onMainComboChanged);
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
}

void QArbitration::onMainComboChanged(int index)
{
    auto request = std::make_shared<rover_msgs::srv::JoyDemuxSetState::Request>();
    request->controller_type = main;
    request->destination = index;
    request->force = true;

    auto result = _clientJoy->async_send_request(request);
}

void QArbitration::onSecComboChanged(int index)
{
    auto request = std::make_shared<rover_msgs::srv::JoyDemuxSetState::Request>();
    request->controller_type = secondary;
    request->destination = index;
    request->force = true;

    auto result = _clientJoy->async_send_request(request);
}