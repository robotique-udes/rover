#ifndef QNAVIGATION_QWAYPOINT_QWAYPOINT_HPP
#define QNAVIGATION_QWAYPOINT_QWAYPOINT_HPP

#include <string>
#include <optional>
#include "json/json.h"

struct sWaypoint
{
    std::string name;
    double latitude;
    double longitude;
    std::string id;
};

class QWaypoint
{
    static constexpr const char* NAVIGATION_PATH = "/Navigation";
    static constexpr const char* JSON_FILE_NAME = "/waypoints.json";

    static constexpr const char* WAYPOINT_JSON = "waypoints";
    static constexpr const char* WAYPOINT_JSON_NAME = "name";
    static constexpr const char* WAYPOINT_JSON_LATITUDE = "latitude";
    static constexpr const char* WAYPOINT_JSON_LONGITUDE = "longitude";
    static constexpr const char* WAYPOINT_JSON_ID = "id";

  public:
    QWaypoint(std::string sessionFolderPath_);
    void registerWaypoint(const sWaypoint& waypoint_);
    

  private:
    void addWaypointToList(const sWaypoint& waypoint_);
    void addWaypointToJson(const sWaypoint& waypoint_);
    void loadWaypointsFromJson(void);
    std::string findLastSessionFolder(void);
    void deleteWaypointFromJson(const std::string& index_);
    void initializeWaypoints();
    std::optional<Json::Value> readJsonFile(const std::string& filePath);
    void writeJsonFile(const std::string& filePath, const Json::Value& root);

    std::string _sessionFolderPath;
};

#endif  // QNAVIGATION_QWAYPOINT_QWAYPOINT_HPP