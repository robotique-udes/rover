#ifndef QMORSECODE_QMORSECODE_HPP
#define QMORSECODE_QMORSECODE_HPP

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/morse_code.hpp>

// QT
#include <QtWidgets/QGridLayout>
#include <QString>
#include "UI_MorseCode.h"
// #include "Worker/QStatusWorker.hpp"

class QMorseCode : public QWidget
{
    Q_OBJECT

    static constexpr const char* TOPIC_MORSE_CODE = "/base/gui/morse_code";

  public:
    QMorseCode(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);

  private:
    void onPbDotClick();
    void onPbDashClick();
    void onPbSpaceClick();
    void sendMorseCode();

    void CB_publishMorseCode();

  private:
    std::shared_ptr<rclcpp::Node> _node;
    Ui::MorseCode _ui;

    // QStatusWorker _QStatusWorker;

    rclcpp::Publisher<rover_msgs::msg::MorseCode>::SharedPtr _pub_morseCode;
};

#endif  // QMORSECODE_QMORSECODE_HPP