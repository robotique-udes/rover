#ifndef SECONDARY_WINDOW_HPP
#define SECONDARY_WINDOW_HPP

#include "QVideoPlayer/QVideoManagerWidget.hpp"
#include <QMainWindow>
#include <QVBoxLayout>
#include <QLabel>
#include "Global/Helpers/QToastNotification.hpp"
#include <QStackedWidget>

class SecondaryWindow : public QMainWindow
{
    Q_OBJECT

  public:
    explicit SecondaryWindow(std::shared_ptr<rclcpp::Node> guiNode_);

  private:
    QWidget _centralWidget;
    QVBoxLayout _layout;
    QStackedWidget _stackedWidget;

    QVideoManagerWidget _videoPlayerWidget;
};

#endif  // SECONDARY_WINDOW_HPP
