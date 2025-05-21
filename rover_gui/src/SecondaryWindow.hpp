#ifndef SECONDARY_WINDOW_HPP
#define SECONDARY_WINDOW_HPP

#include "QVideoPlayer/QVideoManagerWidget.hpp"
#include <QMainWindow>
#include <QVBoxLayout>
#include <QLabel>
#include <QStackedWidget>
#include <QShortcut>

class SecondaryWindow : public QMainWindow
{
    Q_OBJECT

  public:
    explicit SecondaryWindow(std::shared_ptr<rclcpp::Node> guiNode_);

  private:
    void closeEvent(QCloseEvent* event_) override;

    QWidget _centralWidget;
    QVBoxLayout _layout;
    QStackedWidget _stackedWidget;
    QShortcut _closeShortCut;

    QVideoManagerWidget _videoPlayerWidget;
};

#endif  // SECONDARY_WINDOW_HPP