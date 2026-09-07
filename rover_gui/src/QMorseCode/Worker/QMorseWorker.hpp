#ifndef QMORSECODE_WORKER_QMORSE_WORKER_HPP
#define QMORSECODE_WORKER_QMORSE_WORKER_HPP

#include "Global/Workers/QWorker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/morse_code.hpp>
#include <rover_lib2/helpers/macros.hpp>

class QMorseWorker : public QWorker
{
    Q_OBJECT

  public:
    QMorseWorker(QObject* parent_ = nullptr);

    void sendMorseCode(const std::string& morseCodeMsg_);

    void setPublisher(rclcpp::Publisher<rover_msgs::msg::MorseCode>::SharedPtr pub_morseCode_);

  private:
    void sendMorseCodeInternal(const std::string& morseCodeMsg_);

    rclcpp::Publisher<rover_msgs::msg::MorseCode>::SharedPtr _pub_morseCode;
};

#endif  // QMORSECODE_WORKER_QMORSE_WORKER_HPP