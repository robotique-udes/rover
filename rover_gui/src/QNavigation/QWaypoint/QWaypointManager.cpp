#include "QWaypointManager.hpp"

#include <rclcpp/rclcpp.hpp>

#include <rover_lib2/helpers/folders.hpp>
#include <Global/Helpers/QToastNotification/QToastNotification.hpp>

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

std::optional<QList<sWaypoint>> QWaypointManager::loadWaypointsFromJson(void)
{
    std::string filePath = _sessionFolderPath + WAYPOINT_FILE_PATH;
    std::optional<Json::Value> rootOpt = this->readJsonFile(filePath);

    if (!rootOpt.has_value())
    {
        return std::nullopt;
    }

    Json::Value root = rootOpt.value();
    QList<sWaypoint> waypoints;

    if (root.isMember(WAYPOINT_JSON) && root[WAYPOINT_JSON].isArray())
    {
        const Json::Value& waypointsArray = root[WAYPOINT_JSON];

        for (const Json::Value& waypointObj : waypointsArray)
        {
            if (waypointObj.isMember(WAYPOINT_JSON_NAME) && waypointObj.isMember(WAYPOINT_JSON_LATITUDE)
                && waypointObj.isMember(WAYPOINT_JSON_LONGITUDE) && waypointObj.isMember(WAYPOINT_JSON_ID))
            {
                sWaypoint waypoint;
                waypoint.name = waypointObj[WAYPOINT_JSON_NAME].asString();
                waypoint.latitude = waypointObj[WAYPOINT_JSON_LATITUDE].asDouble();
                waypoint.longitude = waypointObj[WAYPOINT_JSON_LONGITUDE].asDouble();
                waypoint.id = waypointObj[WAYPOINT_JSON_ID].asString();

                // this->addWaypointToList(waypoint);
                waypoints.append(waypoint);
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("GUI"), "Loaded %d waypoint(s) from file", waypointsArray.size());
    }

    return waypoints;
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
        QHelper::QToastNotification::getInstance().notifyFromAnyThread("No waypoints found",
                                                                       "Corrupted file. Unable to find waypoint inside JSON",
                                                                       QHelper::QToastNotification::eNotifType::ERROR);
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

std::optional<QList<sWaypoint>> QWaypointManager::initializeWaypoints()
{
    std::string currentFilePath = _sessionFolderPath + WAYPOINT_FILE_PATH;
    if (!std::filesystem::exists(currentFilePath))
    {
        std::string lastSessionFolderPath = this->findLastSessionFolder();
        std::string lastFilePath = lastSessionFolderPath + WAYPOINT_FILE_PATH;
        if (!std::filesystem::exists(lastFilePath))
        {
            RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Unable to load waypoint from JSON. File missing or invalid.");
            return std::nullopt;
        }
        std::filesystem::copy_file(lastFilePath, currentFilePath);
    }
    return this->loadWaypointsFromJson();
}

std::optional<Json::Value> QWaypointManager::readJsonFile(const std::string& filePath)
{
    std::ifstream file(filePath);
    if (!file.is_open())
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Unable to open JSON file: %s", filePath.c_str());
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

void QWaypointManager::syncWaypoints(QString& waypointList_)
{

}