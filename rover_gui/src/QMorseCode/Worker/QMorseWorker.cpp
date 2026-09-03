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

    rover_msgs::msg::MorseCode msg;
    msg.length = len;

    for (uint8_t i = 0; i < len; ++i)
    {
        uint8_t c = static_cast<uint8_t>(morseCode_[i]);
        runningChecksum = static_cast<uint8_t>(runningChecksum + c);  // wraps naturally at 256

        msg.start = (i == 0);
        msg.index = i;
        msg.character = c;
        msg.checksum = runningChecksum;

        _pub_morseCode->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}