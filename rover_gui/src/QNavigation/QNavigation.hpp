#ifndef QNAVIGATION_HPP
#define QNAVIGATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/gps.hpp>
#include "UI_Navigation.h"

#include <QWebChannel>
#include <QListWidgetItem>

class QNavigation : public QWidget
{
    Q_OBJECT

  public:
    QNavigation(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_ = nullptr);


    struct sWaypoint
    {
        QString name;
        double latitude;
        double longitude;
        QString id;
    };

  signals:
    void gpsCallback(double latitude_, double longitude_, double heading_);
    void sendGoal(QString name_, double latitude_, double longitude_, QString id_);
    void calculatePath(double destLat_, double destLon_, QString waypointId_);
    void waypointIsVisible(QString waypointId_, bool visibility_);
    void jsReady(void);
    void clearWaypoints(void);
    void clearPath(void);
    void deleteWaypoint(QString waypointId_);
    void addWaypoint(QString name_, double latitude_, double longitude_, QString id_);

  public slots:
    void pathDistanceCalculated(double distanceMeters_);
    void waypointCreated(QString name_, double latitude_, double longitude_, QString id_);
    void onCalculatePathClicked(void);
    void onWaypointVisibilityChanged(QListWidgetItem* item_);
    void onWaypointSelected(QListWidgetItem* item_);
    void onClearWaypointsClicked(void);
    void onClearPathClicked(void);
    void onDeleteWaypointClicked(void);
    void onSetGoalClicked(void);
    void onWebViewLoadFinished(bool ok);
    void onGpsMessage(const rover_msgs::msg::Gps& msg_);

  private:
    void addWaypointToList(const sWaypoint waypoint_);
    void createNavigationFolder(void);
    void addWaypointToJson(const sWaypoint waypoint_);
    void loadWaypointsFromJson(void);
    std::string findLastSessionFolder(void);
    bool checkFilePath(std::string filePath_);

    QWebChannel _webChannel;
    std::string _sessionFolderPath;
    std::shared_ptr<rclcpp::Node> _node;
    Ui::Navigation _ui;

    rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr _gpsSub;

    QList<sWaypoint> _waypoints;
};

#endif  // QNAVIGATION_HPP
