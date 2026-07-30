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

    this->connect(this, &QMorseCode::morseIsBusy, this, &QMorseCode::onMorseIsBusy);

    _pub_morseCode = this->_node->create_publisher<rover_msgs::msg::MorseCode>(TOPIC_MORSE_CODE, QOS_DEFAULT);

    _sub_morseStatus
        = this->_node->create_subscription<rover_msgs::msg::MorseStatus>(TOPIC_MORSE_STATUS,
                                                                       QOS_DEFAULT,
                                                                       [this](const rover_msgs::msg::MorseStatus& msg_)
                                                                       {
                                                                           emit this->morseIsBusy(msg_);
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

void QMorseCode::onMorseIsBusy(const rover_msgs::msg::MorseStatus& msg_)
{
    this->_ui.lineEdit->setDisabled(msg_.is_busy);
    this->_ui.pb_dash->setDisabled(msg_.is_busy);
    this->_ui.pb_dot->setDisabled(msg_.is_busy);
    this->_ui.pb_send->setDisabled(msg_.is_busy);
    this->_ui.pb_space->setDisabled(msg_.is_busy);

    if (this->_wasBusy != msg_.is_busy)
    {
        _wasBusy = msg_.is_busy;
        this->_ui.lineEdit->clear();
    }

    if (msg_.is_busy)
    {
        this->_ui.lineEdit->setText("MORSE CURRENTLY BUSY");
    }
}

void QMorseCode::sendMorseCode()
{
    std::string morseCode = this->_ui.lineEdit->text().toStdString();
    uint8_t len = static_cast<uint8_t>(morseCode.length());
    uint8_t runningChecksum = 0;

    rover_msgs::msg::MorseCode msg;
    msg.length = len;

    for (uint8_t i = 0; i < len; ++i)
    {
        uint8_t c = static_cast<uint8_t>(morseCode[i]);
        runningChecksum = static_cast<uint8_t>(runningChecksum + c);  // wraps naturally at 256

        msg.start = (i == 0);
        msg.index = i;
        msg.character = c;
        msg.checksum = runningChecksum;

        _pub_morseCode->publish(msg);
        RCLCPP_DEBUG(this->_node->get_logger(), "Sending char %d/%d: %c", i + 1, len, c);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    this->_ui.lineEdit->clear();
}