#include "QArbitration.hpp"
#include <rover_msgs/msg/detail/joy_demux_status__struct.hpp>

QArbitration::QArbitration(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    _ui.setupUi(this);

    this->initComboBoxItems();
}

void QArbitration::initComboBoxItems()
{
    this->_ui.mainComboBox->addItem("None", 0);
    this->_ui.mainComboBox->addItem("Drive Train", 1);
    this->_ui.mainComboBox->addItem("Arm", 2);
    this->_ui.mainComboBox->addItem("Antenna", 3);

    this->_ui.secComboBox->addItem("None", 0);
    this->_ui.secComboBox->addItem("Drive Train", 1);
    this->_ui.secComboBox->addItem("Arm", 2);
    this->_ui.secComboBox->addItem("Antenna", 3);
}