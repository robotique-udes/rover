#include "QWaypoint.hpp"

#include <rclcpp/rclcpp.hpp>

QWaypoint::QWaypoint(std::string sessionFolderPath_) : _sessionFolderPath(sessionFolderPath_)
{
}

void QWaypoint::addWaypointToList(const sWaypoint& waypoint_)
{
    std::string filePath = _sessionFolderPath + JSON_FILE_NAME;
    Json::Value root;
    Json::Value waypointsArray(Json::arrayValue);

    std::optional<Json::Value> rootOpt = this->readJsonFile(filePath);

    if (!rootOpt.has_value())
    {
        root[WAYPOINT_JSON] = Json::arrayValue;
    }
    else
    {
        root = rootOpt.value();
    }

    waypointsArray = root[WAYPOINT_JSON];

    Json::Value waypointObj;
    waypointObj[WAYPOINT_JSON_NAME] = waypoint_.name;
    waypointObj[WAYPOINT_JSON_LATITUDE] = waypoint_.latitude;
    waypointObj[WAYPOINT_JSON_LONGITUDE] = waypoint_.longitude;
    waypointObj[WAYPOINT_JSON_ID] = waypoint_.id;

    waypointsArray.append(waypointObj);

    root[WAYPOINT_JSON] = waypointsArray;
    RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Attempting to write to: %s", filePath.c_str());
    this->writeJsonFile(filePath, root);
}

void QWaypoint::addWaypointToJson(const sWaypoint& waypoint_)
{

}

void QWaypoint::loadWaypointsFromJson(void)
{

}

std::string QWaypoint::findLastSessionFolder(void)
{

}

void QWaypoint::deleteWaypointFromJson(const std::string& index_)
{

}

void QWaypoint::initializeWaypoints()
{

}

std::optional<Json::Value> QWaypoint::readJsonFile(const std::string& filePath)
{

}

void QWaypoint::writeJsonFile(const std::string& filePath, const Json::Value& root)
{

}