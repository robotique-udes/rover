#include "QNavigation.hpp"

// QT
#include <QTimer>
#include "Global/Helpers/QHelpers.hpp"
#include <Global/Helpers/QToastNotification/QToastNotification.hpp>
#include "Global/Helpers/QSessionFolderManager/QSessionFolderManager.hpp"

// Helpers
#include <rover_lib2/helpers/folders.hpp>

#include <fstream>
#include <vector>
#include <algorithm>
#include <filesystem>

constexpr const char* QRC_PATH_MAP_HTML = "qrc:/other/map.html";
constexpr const char* GPS_TOPIC_NAME = "/rover/gps/position";
constexpr const char* NAVIGATION_PATH = "/Navigation";
constexpr const char* JSON_FILE_NAME = "/waypoints.json";

// Default to Studio de Création
constexpr double DEFAULT_LATITUDE = 45.377755;
constexpr double DEFAULT_LONGITUDE = -71.924652;
constexpr double DEFAULT_HEADING = 0.0;

QNavigation::QNavigation(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _webChannel(this),
    _node(guiNode_)
{
    _ui.setupUi(this);
    this->createNavigationFolder();
    this->initializeWaypoints();

    qInstallMessageHandler(
        [](QtMsgType, const QMessageLogContext&, const QString&)
        {
        });

    _ui.webViewContainer->load(QUrl(QRC_PATH_MAP_HTML));

    connect(_ui.webViewContainer, &QWebEngineView::loadFinished, this, &QNavigation::onWebViewLoadFinished);
    connect(_ui.setGoalButton, &QPushButton::clicked, this, &QNavigation::onSetGoalClicked);
    connect(_ui.calculatePathButton, &QPushButton::clicked, this, &QNavigation::onCalculatePathClicked);
    connect(_ui.waypointList, &QListWidget::itemClicked, this, &QNavigation::onWaypointSelected);
    connect(_ui.waypointList, &QListWidget::itemChanged, this, &QNavigation::onWaypointVisibilityChanged);
    connect(_ui.clearWaypointsButton, &QPushButton::clicked, this, &QNavigation::onClearWaypointsClicked);
    connect(_ui.clearPathButton, &QPushButton::clicked, this, &QNavigation::onClearPathClicked);
    connect(_ui.deleteWaypointButton, &QPushButton::clicked, this, &QNavigation::onDeleteWaypointClicked);

    _gpsSub = _node->create_subscription<rover_msgs::msg::Gps>(GPS_TOPIC_NAME,
                                                               1,
                                                               [this](const rover_msgs::msg::Gps& gpsMsg_)
                                                               {
                                                                   this->onGpsMessage(gpsMsg_);
                                                               });

    // Hack | Todo: java script should send a signal when it's ready to update it's position
    QTimer::singleShot(1'500,
                       [this]()
                       {
                           emit this->gpsCallback(DEFAULT_LATITUDE, DEFAULT_LONGITUDE, DEFAULT_HEADING);
                       });
}

void QNavigation::onJsBridgeReady()
{
    for (const sWaypoint& waypoint : _waypoints)
    {
        emit sendGoal(waypoint.name, waypoint.latitude, waypoint.longitude, waypoint.id, false);
    }
}

void QNavigation::createNavigationFolder(void)
{
    std::optional<std::string> optionalSessionFolderPath = QSessionFolderManager::getInstance().getSessionFolderPath();
    std::string homePath;
    std::string sessionPath;

    if (optionalSessionFolderPath.has_value())
    {
        sessionPath = *optionalSessionFolderPath;
        if (sessionPath.empty())
        {
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Empty session folder path",
                                                                           "SessionFolderManager returned an empty path",
                                                                           QHelper::QToastNotification::eNotifType::ERROR);
        }
    }
    else
    {
        QHelper::QToastNotification::getInstance().notifyFromAnyThread("No session folder found",
                                                                       "SessionFolderManager couldn't return a valid path",
                                                                       QHelper::QToastNotification::eNotifType::ERROR);
    }

    std::optional<std::string> optionalHomePath = Folders::getHome();
    if (optionalHomePath.has_value())
    {
        homePath = *optionalHomePath;
        if (homePath.empty())
        {
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Empty home path",
                                                                           "HomePath returned an empty path",
                                                                           QHelper::QToastNotification::eNotifType::ERROR);
        }
    }
    else
    {
        QHelper::QToastNotification::getInstance().notifyFromAnyThread("No home found",
                                                                       "HomePath couldn't return a valid path",
                                                                       QHelper::QToastNotification::eNotifType::ERROR);
    }

    _sessionFolderPath = homePath + sessionPath + NAVIGATION_PATH;
    Folders::createFolder(_sessionFolderPath);
    RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Current navigation folder path: %s", _sessionFolderPath.c_str());
}

void QNavigation::onGpsMessage(const rover_msgs::msg::Gps& msg_)
{
    emit this->gpsCallback(msg_.latitude, msg_.longitude, msg_.heading);
}

void QNavigation::onSetGoalClicked()
{
    if (_ui.inputName->text().isEmpty() || _ui.inputLatitude->text().isEmpty() || _ui.inputLongitude->text().isEmpty())
    {
        QHelper::QPopUp::sendQuestionPopUp("Input Error", "Please enter waypoint name and coordinates.");
        return;
    }

    sWaypoint waypoint;
    waypoint.latitude = _ui.inputLatitude->text().toDouble();
    waypoint.longitude = _ui.inputLongitude->text().toDouble();
    waypoint.name = _ui.inputName->text();

    for (const sWaypoint& waypointIt : _waypoints)
    {
        if (waypointIt.name == waypoint.name)
        {
            QHelper::QPopUp::sendQuestionPopUp("Duplicate Name",
                                               "A waypoint with this name already exists. Please choose a different name.");
            return;
        }
    }

    waypoint.id = "waypoint_" + QUuid::createUuid().toString(QUuid::WithoutBraces);

    this->addWaypointToList(waypoint);
    this->addWaypointToJson(waypoint);
    emit this->sendGoal(waypoint.name, waypoint.latitude, waypoint.longitude, waypoint.id, true);

    _ui.inputName->clear();
    _ui.inputLatitude->clear();
    _ui.inputLongitude->clear();
}

void QNavigation::pathDistanceCalculated(double distanceMeters_)
{
    QString distanceText_;
    if (distanceMeters_ >= 1000.0)
    {
        distanceText_ = QString("%1 km").arg(distanceMeters_ / 1000.0, 0, 'f', 2);
    }
    else
    {
        distanceText_ = QString("%1 m").arg(qRound(distanceMeters_));
    }

    _ui.distanceLabel->setText(distanceText_);
}

void QNavigation::waypointCreated(QString name_, double latitude_, double longitude_, QString id_)
{
    for (const auto& waypoint : _waypoints)
    {
        if (waypoint.id == id_ || waypoint.name == name_)
        {
            return;
        }
    }

    if (id_.isEmpty())
    {
        id_ = "waypoint_" + QUuid::createUuid().toString(QUuid::WithoutBraces);
    }
    sWaypoint waypoint = {name_, latitude_, longitude_, id_};
    this->addWaypointToList(waypoint);
    this->addWaypointToJson(waypoint);
}

void QNavigation::onCalculatePathClicked(void)
{
    QListWidgetItem* currentItem_ = _ui.waypointList->currentItem();
    if (!currentItem_)
    {
        QHelper::QPopUp::sendQuestionPopUp("Select Waypoint", "Please select a waypoint from the list first.");
        return;
    }

    int index_ = _ui.waypointList->row(currentItem_);
    if (index_ >= 0 && index_ < _waypoints.size())
    {
        const sWaypoint& waypoint = _waypoints.at(index_);

        emit this->calculatePath(waypoint.latitude, waypoint.longitude, waypoint.id);
    }
}

void QNavigation::addWaypointToList(const sWaypoint waypoint_)
{
    QString displayText
        = QString("%1 (%2, %3)").arg(waypoint_.name).arg(waypoint_.latitude, 0, 'f', 6).arg(waypoint_.longitude, 0, 'f', 6);

    std::unique_ptr<QListWidgetItem> waypointItem = std::make_unique<QListWidgetItem>(displayText);

    waypointItem->setFlags(waypointItem->flags() | Qt::ItemIsUserCheckable);
    waypointItem->setCheckState(Qt::Checked);
    waypointItem->setData(Qt::UserRole, waypoint_.id);

    _waypoints.append(waypoint_);
    _ui.waypointList->addItem(waypointItem.release());
}

void QNavigation::onWaypointVisibilityChanged(QListWidgetItem* item_)
{
    if (!item_)
    {
        return;
    }

    int index = _ui.waypointList->row(item_);
    if (index >= 0 && index < _waypoints.size())
    {
        const sWaypoint& waypoint = _waypoints.at(index);
        bool isVisible = (item_->checkState() == Qt::Checked);

        emit this->waypointIsVisible(waypoint.id, isVisible);
    }
}

void QNavigation::onWaypointSelected(QListWidgetItem* item_)
{
    if (!item_)
    {
        return;
    }

    int index_ = _ui.waypointList->row(item_);
    if (index_ >= 0 && index_ < _waypoints.size())
    {
        const sWaypoint& waypoint_ = _waypoints.at(index_);

        _ui.inputName->setText(waypoint_.name);
        _ui.inputLatitude->setText(QString::number(waypoint_.latitude, 'f', 6));
        _ui.inputLongitude->setText(QString::number(waypoint_.longitude, 'f', 6));
    }
}

void QNavigation::onDeleteWaypointClicked(void)
{
    QListWidgetItem* currentItem_ = _ui.waypointList->currentItem();
    if (!currentItem_)
    {
        QHelper::QPopUp::sendQuestionPopUp("Select Waypoint", "Please select a waypoint to delete.");
        return;
    }

    int index_ = _ui.waypointList->row(currentItem_);
    if (index_ >= 0 && index_ < _waypoints.size())
    {
        const QString waypointId = _waypoints.at(index_).id;
        this->deleteWaypointFromJson(waypointId);

        delete _ui.waypointList->takeItem(index_);

        emit this->deleteWaypoint(waypointId);

        _waypoints.removeAt(index_);

        _ui.inputName->clear();
        _ui.inputLatitude->clear();
        _ui.inputLongitude->clear();

        _ui.distanceLabel->setText("N/A");

        emit this->clearPath();
    }
}

void QNavigation::onClearWaypointsClicked(void)
{
    QMessageBox::StandardButton result_ = QHelper::QPopUp::sendQuestionPopUp("Clear Waypoints",
                                                                             "Are you sure you want to clear all waypoints?",
                                                                             QMessageBox::Yes | QMessageBox::No);

    if (result_ == QMessageBox::Yes)
    {
        _waypoints.clear();
        _ui.waypointList->clear();

        _ui.inputName->clear();
        _ui.inputLatitude->clear();
        _ui.inputLongitude->clear();
        _ui.distanceLabel->setText("N/A");

        emit this->clearWaypoints();
    }
}

void QNavigation::onWebViewLoadFinished(bool ok_)
{
    if (!ok_)
    {
        return;
    }

    _webChannel.registerObject(QStringLiteral("bridge"), this);
    _ui.webViewContainer->page()->setWebChannel(&_webChannel);

    QString token = qgetenv("CESIUM_TOKEN");
    if (!token.isEmpty())
    {
        _ui.webViewContainer->page()->runJavaScript("Cesium.Ion.defaultAccessToken = '" + token + "';");
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Cesium token not found, can't load map");
    }

    if (!Folders::folderExists(_sessionFolderPath))
    {
        Folders::createFolder(_sessionFolderPath);
    }
}

void QNavigation::onClearPathClicked(void)
{
    _ui.distanceLabel->setText("N/A");

    emit this->clearPath();
}

void QNavigation::addWaypointToJson(const sWaypoint waypoint_)
{
    std::string filePath = _sessionFolderPath + JSON_FILE_NAME;
    Json::Value root;
    Json::Value waypointsArray(Json::arrayValue);

    std::optional<Json::Value> rootOpt = this->readJsonFile(filePath);

    if (!rootOpt.has_value())
    {
        root["waypoints"] = Json::arrayValue;
    }
    else
    {
        root = *rootOpt;
    }


    waypointsArray = root["waypoints"];

    Json::Value waypointObj;
    waypointObj["name"] = waypoint_.name.toStdString();
    waypointObj["latitude"] = waypoint_.latitude;
    waypointObj["longitude"] = waypoint_.longitude;
    waypointObj["id"] = waypoint_.id.toStdString();

    waypointsArray.append(waypointObj);

    root["waypoints"] = waypointsArray;
    RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Attempting to write to: %s", filePath.c_str());
    this->writeJsonFile(filePath, root);
}

void QNavigation::deleteWaypointFromJson(const QString index_)
{
    std::string filePath = _sessionFolderPath + JSON_FILE_NAME;
    Json::Value root;
    std::optional<Json::Value> rootOpt = this->readJsonFile(filePath);

    if (!rootOpt.has_value())
    {
        return;
    }

    root = *rootOpt;

    if (!root.isMember("waypoints") || !root["waypoints"].isArray())
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "No waypoints array found in JSON");
        return;
    }

    Json::Value& waypointsArray = root["waypoints"];
    Json::Value newWaypoints(Json::arrayValue);
    std::string idToRemove = index_.toStdString();

    for (const Json::Value& waypoint : waypointsArray)
    {
        if (!waypoint.isMember("id") || waypoint["id"].asString() != idToRemove)
        {
            newWaypoints.append(waypoint);
        }
    }

    root["waypoints"] = newWaypoints;

    this->writeJsonFile(filePath, root);
}

void QNavigation::initializeWaypoints()
{
    std::string currentFilePath = _sessionFolderPath + JSON_FILE_NAME;
    if (!std::filesystem::exists(currentFilePath))
    {
        std::string lastSessionFolderPath = this->findLastSessionFolder();
        std::string lastFilePath = lastSessionFolderPath + JSON_FILE_NAME;
        if (!std::filesystem::exists(lastFilePath))
        {
            return;
        }
        std::filesystem::copy_file(lastFilePath, currentFilePath);
    }
    this->loadWaypointsFromJson();
}

void QNavigation::loadWaypointsFromJson(void)
{
    std::string filePath = _sessionFolderPath + JSON_FILE_NAME;
    std::optional<Json::Value> rootOpt = this->readJsonFile(filePath);

    if (!rootOpt.has_value())
    {
        return;
    }

    Json::Value root = *rootOpt;

    if (root.isMember("waypoints") && root["waypoints"].isArray())
    {
        const Json::Value& waypointsArray = root["waypoints"];

        for (const Json::Value& waypointObj : waypointsArray)
        {
            if (waypointObj.isMember("name") && waypointObj.isMember("latitude") && waypointObj.isMember("longitude")
                && waypointObj.isMember("id"))
            {
                sWaypoint waypoint;
                waypoint.name = QString::fromStdString(waypointObj["name"].asString());
                waypoint.latitude = waypointObj["latitude"].asDouble();
                waypoint.longitude = waypointObj["longitude"].asDouble();
                waypoint.id = QString::fromStdString(waypointObj["id"].asString());

                this->addWaypointToList(waypoint);
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("GUI"), "Loaded %d waypoints from file", waypointsArray.size());
    }
}

std::string QNavigation::findLastSessionFolder(void)
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

std::optional<Json::Value> QNavigation::readJsonFile(const std::string& filePath)
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

void QNavigation::writeJsonFile(const std::string& filePath, const Json::Value& root)
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