#ifndef __QNAVIGATION_HPP__
#define __QNAVIGATION_HPP__

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/gps.hpp"
#include "UI_Navigation.h"

#include <QWidget>
#include <QWebChannel>
#include <QThread>
#include <QMetaObject>
#include <QListWidgetItem>

// Structure to store waypoint information
struct Waypoint {
    QString name;
    double latitude;
    double longitude;
};

class QNavigation : public QWidget
{
    Q_OBJECT
  public:
    QNavigation(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent = nullptr);
    ~QNavigation();

  signals:
    void gpsCallback(double latitude, double longitude, double heading);
    void sendGoal(QString waypointName, double latitude, double longitude);
    void calculatePath(double latitude, double longitude);
    void jsReady();

  public slots:
    void pathDistanceCalculated(double distanceMeters);
    void onCalculatePathClicked();
    void onWaypointSelected(QListWidgetItem* item);
    void onClearWaypointsClicked();
    void waypointCreated(QString name, double latitude, double longitude);

  private:
    void addWaypointToList(const QString& name, double latitude, double longitude);
    
    QWebChannel* webChannel;
    QLineEdit* _lineEditLatitude;
    QLineEdit* _lineEditLongitude;
    QPushButton* _pushButtonSetGoal;
    QPushButton* _pushButtonCalculatePath;
    QLabel* _labelDistance;

    rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr _gpsSub;

    std::shared_ptr<rclcpp::Node> _node;
    Ui::Navigation* ui;

    double _currentLat = 0.0;
    double _currentLon = 0.0;
    double _currentHeading = 0.0;
    
    QList<Waypoint> _waypoints;
};

#endif  // QNAVIGATION_HPP