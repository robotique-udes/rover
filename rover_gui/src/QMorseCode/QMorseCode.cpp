#include "QMorseCode.hpp"

#include <rover_lib2/helpers/assert.hpp>

#include "rover_lib2/helpers/constants.hpp"

QMorseCode::QMorseCode(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    ASSERT_COND(_node != nullptr);

    _ui.setupUi(this);

    this->connect(_ui.pb_dot,
                  &QPushButton::clicked,
                  this,
                  [this]()
                  {
                      this->onPbDotClick();
                  });

    this->connect(_ui.pb_dash,
                  &QPushButton::clicked,
                  this,
                  [this]()
                  {
                      this->onPbDashClick();
                  });

    this->connect(_ui.pb_space,
                  &QPushButton::clicked,
                  this,
                  [this]()
                  {
                      this->onPbSpaceClick();
                  });

    this->connect(_ui.pb_send,
                  &QPushButton::clicked,
                  this,
                  [this]()
                  {
                      this->sendMorseCode();
                  });

    this->connect(_ui.lineEdit,
                  &QLineEdit::returnPressed,
                  this,
                  [this]()
                  {
                      this->sendMorseCode();
                  });

    _pub_morseCode = this->_node->create_publisher<rover_msgs::msg::MorseCode>(TOPIC_MORSE_CODE, QOS_DEFAULT);
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

void QMorseCode::sendMorseCode()
{
    std::string morseCode = this->_ui.lineEdit->text().toStdString();
    rover_msgs::msg::MorseCode msg;
    msg.speed_wpm = 18;
    msg.symbol = 1;
    _pub_morseCode->publish(msg);

    this->_ui.lineEdit->clear();
}