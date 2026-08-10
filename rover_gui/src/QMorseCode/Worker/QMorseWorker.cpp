#include "QMorseWorker.hpp"

QMorseWorker::QMorseWorker(QObject* parent_):
    QWorker(false, parent_)
{
}

void QMorseWorker::sendMorseCode(const std::string& morseCode_)
{
    this->addTask(
        [this, morseCode_](void)
        {
            if (_pub_morseCode != nullptr)
            {
                this->sendMorseCodeInternal(morseCode_);
            }
        });
}

void QMorseWorker::setPublisher(rclcpp::Publisher<rover_msgs::msg::MorseCode>::SharedPtr pub_morseCode_)
{
    _pub_morseCode = pub_morseCode_;
    this->start();
}

void QMorseWorker::sendMorseCodeInternal(const std::string& morseCode_)
{
    uint8_t len = static_cast<uint8_t>(morseCode_.length());
    uint8_t runningChecksum = 0;

    // One id per morse command, held constant across every frame of this
    // message. Wraps naturally at 256 - the receiver's newer-id comparison
    // tolerates that as long as ~127 messages don't queue up in flight,
    // which can't happen given only one command is ever outstanding.
    const uint8_t msgId = _nextMsgId++;

    rover_msgs::msg::MorseCode msg;
    msg.length = len;
    msg.msg_id = msgId;

    for (uint8_t i = 0; i < len; ++i)
    {
        uint8_t c = static_cast<uint8_t>(morseCode_[i]);
        runningChecksum = static_cast<uint8_t>(runningChecksum + c);  // wraps naturally at 256

        msg.index = i;
        msg.character = c;
        msg.checksum = runningChecksum;

        _pub_morseCode->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}