#include "QMorseCode.hpp"

#include <thread>
#include <chrono>
#include <rover_lib2/helpers/assert.hpp>

#include "rover_lib2/helpers/constants.hpp"

QMorseCode::QMorseCode(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_),
    _QMorseWorker(this)
{
    ASSERT_COND(_node != nullptr);

    _ui.setupUi(this);

    this->connect(_ui.pb_dot, &QPushButton::clicked, this, &QMorseCode::onPbDotClick);

    this->connect(_ui.pb_dash, &QPushButton::clicked, this, &QMorseCode::onPbDashClick);

    this->connect(_ui.pb_space, &QPushButton::clicked, this, &QMorseCode::onPbSpaceClick);

    this->connect(_ui.pb_send, &QPushButton::clicked, this, &QMorseCode::sendMorseCode);

    this->connect(_ui.lineEdit, &QLineEdit::returnPressed, this, &QMorseCode::sendMorseCode);

    this->connect(this, &QMorseCode::morseIsBusy, this, &QMorseCode::onMorseIsBusy);

    this->_pub_morseCode = this->_node->create_publisher<rover_msgs::msg::MorseCode>(TOPIC_MORSE_CODE, QOS_DEFAULT);
    this->_QMorseWorker.setPublisher(this->_pub_morseCode);

    this->_sub_morseStatus
        = this->_node->create_subscription<rover_msgs::msg::MorseStatus>(TOPIC_MORSE_STATUS,
                                                                         QOS_DEFAULT,
                                                                         [this](const rover_msgs::msg::MorseStatus& msg_)
                                                                         {
                                                                             emit this->morseIsBusy(msg_.is_busy);
                                                                         });
}

void QMorseCode::onPbDotClick()
{
    this->_ui.lineEdit->insert(".");
}

void QMorseCode::onPbDashClick()
{
    this->_ui.lineEdit->insert("-");
}

void QMorseCode::onPbSpaceClick()
{
    this->_ui.lineEdit->insert(" ");
}

void QMorseCode::onMorseIsBusy(bool isBusy_)
{
    this->_ui.lineEdit->setDisabled(isBusy_);
    this->_ui.pb_dash->setDisabled(isBusy_);
    this->_ui.pb_dot->setDisabled(isBusy_);
    this->_ui.pb_send->setDisabled(isBusy_);
    this->_ui.pb_space->setDisabled(isBusy_);

    if (this->_wasBusy != isBusy_)
    {
        _wasBusy = isBusy_;
        this->_ui.lineEdit->clear();
    }

    if (isBusy_)
    {
        this->_ui.lineEdit->setText("MORSE CURRENTLY BUSY");
    }
}

void QMorseCode::sendMorseCode()
{
    std::string morseCode = this->_ui.lineEdit->text().toStdString();

    this->_QMorseWorker.sendMorseCode(morseCode);

    this->_ui.lineEdit->clear();
}