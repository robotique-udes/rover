#ifndef QNAVIGATION_HPP
#define QNAVIGATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/gps.hpp>
#include "UI_Navigation.h"

#include <QWebChannel>
#include <QListWidgetItem>
#include <iostream>
#include <optional>

#include <json/json.h>

class QNavigation : public QWidget
{
    Q_OBJECT

  public:
    QNavigation(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_ = nullptr);

    struct sWaypoint
    {
        std::string name;
        double latitude;
        double longitude;
        std::string id;
    };

  signals:
    void gpsCallback(double latitude_, double longitude_, double heading_);
    void sendGoal(const QString& name_, double latitude_, double longitude_, const QString& id_, bool flyTo_);
    void calculatePath(double destLat_, double destLon_, const QString& waypointId_);
    void waypointIsVisible(const QString& waypointId_, bool visibility_);
    void jsReady(void);
    void clearWaypoints(void);
    void clearPath(void);
    void deleteWaypoint(const QString& waypointId_);
    void addWaypoint(const QString& name_, double latitude_, double longitude_, const QString& id_);

  public slots:
    void pathDistanceCalculated(double distanceMeters_);
    void waypointCreated(const QString& name_, double latitude_, double longitude_, QString& id_);
    void onCalculatePathClicked(void);
    void onWaypointVisibilityChanged(QListWidgetItem* item_);
    void onWaypointSelected(QListWidgetItem* item_);
    void onClearWaypointsClicked(void);
    void onClearPathClicked(void);
    void onDeleteWaypointClicked(void);
    void onSetGoalClicked(void);
    void onWebViewLoadFinished(bool ok);
    void onGpsMessage(const rover_msgs::msg::Gps& msg_);
    void onJsBridgeReady(void);

  private:
    void addWaypointToList(const sWaypoint& waypoint_);
    void createNavigationFolder(void);
    void addWaypointToJson(const sWaypoint& waypoint_);
    void loadWaypointsFromJson(void);
    std::string findLastSessionFolder(void);
    void deleteWaypointFromJson(const std::string& index_);
    void initializeWaypoints();
    std::optional<Json::Value> readJsonFile(const std::string& filePath);
    void writeJsonFile(const std::string& filePath, const Json::Value& root);
    void registerWaypoint(const sWaypoint& waypoint_);

    QWebChannel _webChannel;
    std::string _sessionFolderPath;
    std::shared_ptr<rclcpp::Node> _node;
    Ui::Navigation _ui;

    rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr _gpsSub;

    QList<sWaypoint> _waypoints;
};

#endif  // QNAVIGATION_HPP
