#ifndef QMORSECODE_QMORSECODE_HPP
#define QMORSECODE_QMORSECODE_HPP

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/morse_code.hpp>
#include <rover_msgs/msg/morse_status.hpp>

// QT
#include <QtWidgets/QGridLayout>
#include <QString>
#include "UI_MorseCode.h"

class QMorseCode : public QWidget
{
    Q_OBJECT

    static constexpr const char* TOPIC_MORSE_CODE = "/base/gui/morse_code";
    static constexpr const char* TOPIC_MORSE_STATUS = "/base/gui/morse_status";

  public:
    QMorseCode(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);

  private:
    void onPbDotClick();
    void onPbDashClick();
    void onPbSpaceClick();
    void sendMorseCode();
    void CB_publishMorseCode();

  signals:
    void morseIsBusy(bool isBusy_);

  private slots:
    void onMorseIsBusy(bool isBusy_);

  private:
    std::shared_ptr<rclcpp::Node> _node;
    Ui::MorseCode _ui;

    rclcpp::Publisher<rover_msgs::msg::MorseCode>::SharedPtr _pub_morseCode;
    rclcpp::Subscription<rover_msgs::msg::MorseStatus>::SharedPtr _sub_morseStatus;

    bool _wasBusy = false;
};

#endif  // QMORSECODE_QMORSECODE_HPP