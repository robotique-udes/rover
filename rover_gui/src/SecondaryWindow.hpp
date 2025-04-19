#ifndef SECONDARY_WINDOW_HPP
#define SECONDARY_WINDOW_HPP

#include <QMainWindow>
#include <QVBoxLayout>
#include <QLabel>
#include "Global/Helpers/QToastNotification.hpp"

class SecondaryWindow : public QMainWindow
{
  public:
    explicit SecondaryWindow(std::shared_ptr<rclcpp::Node> guiNode_);

  private:
    QWidget _centralWidget;
    QVBoxLayout _layout;

    QLabel _tempLabel;
};

#endif  // SECONDARY_WINDOW_HPP
