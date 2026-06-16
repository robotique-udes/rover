#ifndef QNAVIGATION_HPP
#define QNAVIGATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/gps.hpp>
#include "UI_Navigation.h"

#include "QWaypoint/QWaypointManager.hpp"
#include "Worker/QPathManager.hpp"

#include <QWebChannel>
#include <QListWidgetItem>

#include <iostream>
#include <optional>
#include <vector>
#include <json/json.h>

class QNavigation : public QWidget
{
    Q_OBJECT

  public:
    QNavigation(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_ = nullptr);

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
    void writePosToCSV(double latitude_, double longitude_);
    void updatePathTaken(double latitude_, double longitude_);
    void loadFullPath(const QVariantList& oldPath_);

  public slots:
    void pathDistanceCalculated(double distanceMeters_, double heading_);
    void waypointCreated(const QString& name_, double latitude_, double longitude_, const QString& id_);
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
    void addWaypointToUI(const sWaypoint& waypoint);
    void initializeWaypointManager(void);
    void initializePathManager(void);
    void createNavigationFolder(void);
    void onCSVWriteTimer(void);

    QWebChannel _webChannel;
    std::shared_ptr<rclcpp::Node> _node;
    rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr _gpsSub;
    rclcpp::TimerBase::SharedPtr _csvWriteTimer;

    Ui::Navigation _ui;
    QList<sWaypoint> _waypointsList;
    QVariantList _oldPath;
    rover_msgs::msg::Gps _latestGpsMsg;
    bool _hasGpsData = false;

    QWaypointManager _waypointManager;
    QPathManager _pathManager;
    std::string _sessionFolderPath;
};

#endif  // QNAVIGATION_HPP
