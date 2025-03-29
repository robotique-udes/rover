#ifndef SECONDARY_WINDOW_HPP
#define SECONDARY_WINDOW_HPP

#include "QVideoPlayer/QVideoPlayer.hpp"
#include <QMainWindow>
#include <QVBoxLayout>
#include <QLabel>
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

    QLabel _tempLabel;
    QVideoPlayer _videoPlayerWidget;
};

#endif  // SECONDARY_WINDOW_HPP
