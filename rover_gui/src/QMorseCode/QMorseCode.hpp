#ifndef QMORSECODE_QMORSECODE_HPP
#define QMORSECODE_QMORSECODE_HPP

// ROS
#include <rclcpp/rclcpp.hpp>

// QT
#include <QtWidgets/QGridLayout>
#include <QLabel>
#include "UI_MorseCode.h"
// #include "Worker/QStatusWorker.hpp"

class QMorseCode : public QWidget
{
    Q_OBJECT

  public:
    QMorseCode(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);

  private:
    std::shared_ptr<rclcpp::Node> _node;
    Ui::MorseCode _ui;

    // QStatusWorker _QStatusWorker;
};

#endif  // QMORSECODE_QMORSECODE_HPP