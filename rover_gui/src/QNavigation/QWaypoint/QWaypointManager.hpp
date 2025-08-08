#ifndef QNAVIGATION_QWAYPOINT_QWAYPOINTMANAGER_HPP
#define QNAVIGATION_QWAYPOINT_QWAYPOINTMANAGER_HPP

#include <string>
#include <optional>
#include "json/json.h"
#include <QListWidgetItem>

struct sWaypoint
{
    std::string name;
    double latitude;
    double longitude;
    std::string id;
};

class QWaypointManager
{
    static constexpr const char* NAVIGATION_PATH = "/Navigation";
    static constexpr const char* WAYPOINT_FILE_PATH = "/waypoints.json";

    static constexpr const char* WAYPOINT_JSON = "waypoints";
    static constexpr const char* WAYPOINT_JSON_NAME = "name";
    static constexpr const char* WAYPOINT_JSON_LATITUDE = "latitude";
    static constexpr const char* WAYPOINT_JSON_LONGITUDE = "longitude";
    static constexpr const char* WAYPOINT_JSON_ID = "id";

  public:
    QWaypointManager();
    std::optional<QList<sWaypoint>> initializeWaypoints();
    void setSessionFolderPath(const std::string& sessionFolderPath_);

    void addWaypointToJson(const sWaypoint& waypoint_);
    void deleteWaypointFromJson(const std::string& index_);
    

  private:
    std::optional<QList<sWaypoint>> loadWaypointsFromJson(void);
    std::string findLastSessionFolder(void);
    std::optional<Json::Value> readJsonFile(const std::string& filePath);
    void writeJsonFile(const std::string& filePath, const Json::Value& root);

    std::string _sessionFolderPath;
};

#endif  // QNAVIGATION_QWAYPOINT_QWAYPOINTMANAGER_HPP