#ifndef __QVIDEO_PLAYER_HPP__
#define __QVIDEO_PLAYER_HPP__

#include "rclcpp/rclcpp.hpp"
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>

#include "QAruco.hpp"

class QVideoPlayer : public QWidget
{
    Q_OBJECT

  public:
    QVideoPlayer(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
        QWidget(parent_),
        _node(guiNode_),
        _dashboardLayout(this),
        _videoFrameWidget(guiNode_, this)
    {
        this->setLayout(&_dashboardLayout);

        // Add your dashboard widget here
        _dashboardLayout.addWidget(&_videoFrameWidget);
    }

  private:
    std::shared_ptr<rclcpp::Node> _node;

    QGridLayout _dashboardLayout;
    QAruco _videoFrameWidget;
};

#endif  // QDASHBOARD_HPP