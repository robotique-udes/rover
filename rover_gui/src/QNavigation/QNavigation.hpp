#ifndef __QNAVIGATION_HPP__
#define __QNAVIGATION_HPP__

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/gps.hpp"
#include "UI_Navigation.h"

#include <QWidget>
#include <QWebChannel>
#include <QThread>
#include <QMetaObject>

class QNavigation : public QWidget
{
    Q_OBJECT
  public:
    QNavigation(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent = nullptr);
    ~QNavigation();

  signals:
    void gpsCallback(double latitude, double longitude, double heading);
    void sendGoal(QString waypointName, double latitude, double longitude);

  private:
    QWebChannel* webChannel;
    QLineEdit* _lineEditLatitude;
    QLineEdit* _lineEditLongitude;
    QPushButton* _pushButtonSetGoal;

    rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr _gpsSub;

    std::shared_ptr<rclcpp::Node> _node;
    Ui::Navigation* ui;

};

#endif  // QNAVIGATION_HPP
