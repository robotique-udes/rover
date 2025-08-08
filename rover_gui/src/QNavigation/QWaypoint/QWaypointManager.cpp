#include "QWaypointManager.hpp"

#include <rclcpp/rclcpp.hpp>

#include <rover_lib2/helpers/folders.hpp>

#include <fstream>
#include <vector>
#include <algorithm>
#include <filesystem>

QWaypointManager::QWaypointManager() {}

void QWaypointManager::setSessionFolderPath(const std::string& sessionFolderPath_)
{
    if (!sessionFolderPath_.empty())
    {
        _sessionFolderPath = sessionFolderPath_;
    }
}

void QWaypointManager::addWaypointToJson(const sWaypoint& waypoint_)
{
    std::string filePath = _sessionFolderPath + WAYPOINT_FILE_PATH;
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

std::string QWaypointManager::findLastSessionFolder(void)
{
    std::filesystem::path currentPath(_sessionFolderPath);
    std::filesystem::path sessionBasePath = currentPath.parent_path().parent_path();

    if (!std::filesystem::exists(sessionBasePath))
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Session base path doesn't exist: %s", sessionBasePath.c_str());
        return "";
    }

    std::vector<std::string> sessionFolders;
    for (const auto& entry : std::filesystem::directory_iterator(sessionBasePath))
    {
        if (entry.is_directory())
        {
            sessionFolders.push_back(entry.path().string());
        }
    }

    std::sort(sessionFolders.begin(), sessionFolders.end(), std::greater<std::string>());

    if (sessionFolders.size() > 1)
    {
        std::string lastSessionFolderPath = sessionFolders[1] + NAVIGATION_PATH;
        RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Found latest session folder: %s", lastSessionFolderPath.c_str());
        return lastSessionFolderPath;
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Couldn't find last session");
        return "";
    }
}

void QWaypointManager::deleteWaypointFromJson(const std::string& index_)
{
    std::string filePath = _sessionFolderPath + WAYPOINT_FILE_PATH;
    Json::Value root;
    std::optional<Json::Value> rootOpt = this->readJsonFile(filePath);

    if (!rootOpt.has_value())
    {
        return;
    }

    root = rootOpt.value();

    if (!root.isMember(WAYPOINT_JSON) || !root[WAYPOINT_JSON].isArray())
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "No waypoints array found in JSON");
        return;
    }

    Json::Value& waypointsArray = root[WAYPOINT_JSON];
    Json::Value newWaypoints(Json::arrayValue);
    std::string idToRemove = index_;

    for (const Json::Value& waypoint : waypointsArray)
    {
        if (!waypoint.isMember(WAYPOINT_JSON_ID) || waypoint[WAYPOINT_JSON_ID].asString() != idToRemove)
        {
            newWaypoints.append(waypoint);
        }
    }

    root[WAYPOINT_JSON] = newWaypoints;

    this->writeJsonFile(filePath, root);
}

void QWaypointManager::initializeWaypoints()
{
    std::string currentFilePath = _sessionFolderPath + WAYPOINT_FILE_PATH;
    if (!std::filesystem::exists(currentFilePath))
    {
        std::string lastSessionFolderPath = this->findLastSessionFolder();
        std::string lastFilePath = lastSessionFolderPath + WAYPOINT_FILE_PATH;
        if (!std::filesystem::exists(lastFilePath))
        {
            RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Unable to load waypoint from JSON. File missing or invalid.");
            return;
        }
        std::filesystem::copy_file(lastFilePath, currentFilePath);
    }

    std::optional<Json::Value> rootOpt = this->readJsonFile(currentFilePath);
    if (!rootOpt.has_value())
    {
        return;
    }

    Json::Value root = rootOpt.value();
    Json::Value waypointsArray = root[WAYPOINT_JSON];

    RCLCPP_INFO(rclcpp::get_logger("GUI"), "Loaded %d waypoint(s) from file", waypointsArray.size());
}

std::optional<Json::Value> QWaypointManager::readJsonFile(const std::string& filePath)
{
    std::ifstream file(filePath);
    if (!file.is_open())
    {
        RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Unable to open JSON file: %s", filePath.c_str());
        return std::nullopt;
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errors;
    if (!Json::parseFromStream(builder, file, &root, &errors))
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Failed to parse JSON file: %s", errors.c_str());
        return std::nullopt;
    }
    return root;
}

void QWaypointManager::writeJsonFile(const std::string& filePath, const Json::Value& root)
{
    std::ofstream outputFile(filePath);
    if (!outputFile.is_open())
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Failed to open file for writing: %s", filePath.c_str());
        return;
    }
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "  ";
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    writer->write(root, &outputFile);
    outputFile.close();
}

void QWaypointManager::syncWaypoints(QList<sWaypoint>& waypointList_)
{
    Json::Value root;
    std::string filePath = _sessionFolderPath + WAYPOINT_FILE_PATH;
    std::optional<Json::Value> rootOpt = this->readJsonFile(filePath);
    if (!rootOpt.has_value())
    {
        root[WAYPOINT_JSON] = Json::arrayValue;
    }
    else
    {
        root = rootOpt.value();
        if (!root.isMember(WAYPOINT_JSON) || !root[WAYPOINT_JSON].isArray())
        {
            root[WAYPOINT_JSON] = Json::arrayValue;
        }
    }
    Json::Value waypointsArray = root[WAYPOINT_JSON];

    std::set<std::string> jsonIds = this->getJsonIds(waypointsArray);
    std::set<std::string> listIds = this->getListIds(waypointList_);

    this->addMissingWaypointsToList(listIds, waypointList_, waypointsArray);
    this->addMissingWaypointsToJson(jsonIds, waypointList_, waypointsArray);

    root[WAYPOINT_JSON] = waypointsArray;
    this->writeJsonFile(filePath, root);
}

std::set<std::string> QWaypointManager::getJsonIds(const Json::Value& waypointsArray_)
{
    std::set<std::string> ids;
    for (const Json::Value& waypoint : waypointsArray_)
    {
        ids.insert(waypoint[WAYPOINT_JSON_ID].asString());
    }

    return ids;
}

std::set<std::string> QWaypointManager::getListIds(const QList<sWaypoint>& waypointsList_)
{
    std::set<std::string> ids;
    for (const sWaypoint& waypoint : waypointsList_)
    {
        ids.insert(waypoint.id);
    }

    return ids;
}

void QWaypointManager::addMissingWaypointsToList(std::set<std::string>& listIds_,
                                                 QList<sWaypoint>& waypointsList_,
                                                 const Json::Value& waypointsArray_)
{
    for (const Json::Value& jsonWaypoint : waypointsArray_)
    {
        if (jsonWaypoint.isMember(WAYPOINT_JSON_ID))
        {
            std::string jsonId = jsonWaypoint[WAYPOINT_JSON_ID].asString();
            if (listIds_.find(jsonId) == listIds_.end())
            {
                if (jsonWaypoint.isMember(WAYPOINT_JSON_NAME) && jsonWaypoint.isMember(WAYPOINT_JSON_LATITUDE)
                    && jsonWaypoint.isMember(WAYPOINT_JSON_LONGITUDE))
                {
                    sWaypoint waypoint;
                    waypoint.name = jsonWaypoint[WAYPOINT_JSON_NAME].asString();
                    waypoint.latitude = jsonWaypoint[WAYPOINT_JSON_LATITUDE].asDouble();
                    waypoint.longitude = jsonWaypoint[WAYPOINT_JSON_LONGITUDE].asDouble();
                    waypoint.id = jsonId;

                    waypointsList_.append(waypoint);
                    RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Added waypoint to list: %s", waypoint.name.c_str());
                }
            }
        }
    }
}

void QWaypointManager::addMissingWaypointsToJson(std::set<std::string>& jsonIds_,
                                                 const QList<sWaypoint>& waypointsList_,
                                                 Json::Value& waypointsArray_)
{
    for (const sWaypoint& listWaypoint : waypointsList_)
    {
        if (jsonIds_.find(listWaypoint.id) == jsonIds_.end())
        {
            Json::Value waypointObj;
            waypointObj[WAYPOINT_JSON_NAME] = listWaypoint.name;
            waypointObj[WAYPOINT_JSON_LATITUDE] = listWaypoint.latitude;
            waypointObj[WAYPOINT_JSON_LONGITUDE] = listWaypoint.longitude;
            waypointObj[WAYPOINT_JSON_ID] = listWaypoint.id;

            waypointsArray_.append(waypointObj);
            RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Added waypoint to JSON: %s", listWaypoint.name.c_str());
        }
    }
}

void QWaypointManager::clearWaypoints()
{
    Json::Value root;
    root[WAYPOINT_JSON] = Json::arrayValue;

    std::string filePath = _sessionFolderPath + WAYPOINT_FILE_PATH;

    this->writeJsonFile(filePath, root);
}