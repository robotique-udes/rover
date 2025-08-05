#include "QNavigation.hpp"

#include "Global/Helpers/QHelpers.hpp"
#include <Global/Helpers/QToastNotification/QToastNotification.hpp>
#include "Global/Helpers/QSessionFolderManager/QSessionFolderManager.hpp"
#include <rover_lib2/helpers/folders.hpp>
#include <QTimer>
#include <json/json.h>
#include <fstream>

constexpr const char* QRC_PATH_MAP_HTML = "qrc:/other/map.html";
constexpr const char* GPS_TOPIC_NAME = "/rover/gps/position";
constexpr const char* NAVIGATION_PATH = "/Navigation";
constexpr const char* FILE_NAME = "/waypoints.json";

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

    

    if (this->createNavigationFolder())
    {
        this->waypointsFromJson();
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Unable to create navigation folder. No waypoints found");
    }
}

void QNavigation::closeEvent(QCloseEvent* event)
{
    this->addWaypointsToJson();
    QWidget::closeEvent(event);
}

bool QNavigation::createNavigationFolder(void)
{
    bool success = false;
    std::optional<std::string> optionalSessionFolderPath = QSessionFolderManager::getInstance().getSessionFolderPath();
    std::string homePath;
    std::string sessionPath;

    if (optionalSessionFolderPath.has_value())
    {
        sessionPath = *optionalSessionFolderPath;
        if (sessionPath.empty())
        {
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("No session folder found",
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
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("No session folder found",
                                                                           "HomePath returned an empty path",
                                                                           QHelper::QToastNotification::eNotifType::ERROR);
        }
    }
    else
    {
        QHelper::QToastNotification::getInstance().notifyFromAnyThread("No session folder found",
                                                                       "HomePath couldn't return a valid path",
                                                                       QHelper::QToastNotification::eNotifType::ERROR);
    }

    _sessionFolderPath = homePath + sessionPath + NAVIGATION_PATH;
    RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Current navigation folder path: %s", _sessionFolderPath.c_str());
    if (!Folders::folderExists(_sessionFolderPath))
    {
        success = Folders::createFolder(_sessionFolderPath);
    }

    return success;
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

    double lat = _ui.inputLatitude->text().toDouble();
    double lon = _ui.inputLongitude->text().toDouble();
    QString name = _ui.inputName->text();

    for (const sWaypoint& waypoint : _waypoints)
    {
        if (waypoint.name == name)
        {
            QHelper::QPopUp::sendQuestionPopUp("Duplicate Name",
                                               "A waypoint with this name already exists. Please choose a different name.");
            return;
        }
    }

    QString id = "waypoint_" + QUuid::createUuid().toString(QUuid::WithoutBraces);

    this->addWaypointToList(name, lat, lon, id);

    emit this->sendGoal(name, lat, lon, id);

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

    this->addWaypointToList(name_, latitude_, longitude_, id_);
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

void QNavigation::addWaypointToList(const QString& name_, double latitude_, double longitude_, const QString& id_)
{
    sWaypoint waypoint_;
    waypoint_.name = name_;
    waypoint_.latitude = latitude_;
    waypoint_.longitude = longitude_;
    waypoint_.id = id_;

    QString displayText = QString("%1 (%2, %3)").arg(name_).arg(latitude_, 0, 'f', 6).arg(longitude_, 0, 'f', 6);

    std::unique_ptr<QListWidgetItem> waypointItem = std::make_unique<QListWidgetItem>(displayText);

    waypointItem->setFlags(waypointItem->flags() | Qt::ItemIsUserCheckable);
    waypointItem->setCheckState(Qt::Checked);
    waypointItem->setData(Qt::UserRole, id_);

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
        const QString waypointId_ = _waypoints.at(index_).id;

        delete _ui.waypointList->takeItem(index_);

        emit this->deleteWaypoint(waypointId_);

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
        RCLCPP_WARN(_node->get_logger(), "Cesium token not found, can't load map");
    }
}

void QNavigation::onClearPathClicked(void)
{
    _ui.distanceLabel->setText("N/A");

    emit this->clearPath();
}

void QNavigation::addWaypointsToJson(void)
{
    if (_waypoints.isEmpty())
    {
        return;
    }

    Json::Value root;
    Json::Value waypointsArray(Json::arrayValue);

    for (const sWaypoint& waypoint : _waypoints)
    {
        Json::Value waypointObj;
        waypointObj["name"] = waypoint.name.toStdString();
        waypointObj["latitude"] = waypoint.latitude;
        waypointObj["longitude"] = waypoint.longitude;
        waypointObj["id"] = waypoint.id.toStdString();

        waypointsArray.append(waypointObj);
    }

    root["waypoints"] = waypointsArray;
    std::string filePath = _sessionFolderPath + NAVIGATION_PATH;  // Adjust path as needed
    std::ofstream file(filePath);

    if (file.is_open())
    {
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "  ";  // Pretty print with 2 spaces
        std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        writer->write(root, &file);
        file.close();

        RCLCPP_INFO(_node->get_logger(), "Waypoints saved to %s", filePath.c_str());
    }
    else
    {
        RCLCPP_ERROR(_node->get_logger(), "Failed to open file for writing: %s", filePath.c_str());
    }
}

void QNavigation::waypointsFromJson(void)
{
    std::string filePath = "/home/anibal/ros2_ws/src/rover/rover_gui/src/QNavigation/waypoints.json";
    std::ifstream file(filePath);

    if (!file.is_open())
    {
        RCLCPP_INFO(_node->get_logger(), "No waypoints file found at %s", filePath.c_str());
        return;
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errors;

    if (Json::parseFromStream(builder, file, &root, &errors))
    {
        if (root.isMember("waypoints") && root["waypoints"].isArray())
        {
            const Json::Value& waypointsArray = root["waypoints"];

            for (const Json::Value& waypointObj : waypointsArray)
            {
                if (waypointObj.isMember("name") && waypointObj.isMember("latitude") && waypointObj.isMember("longitude")
                    && waypointObj.isMember("id"))
                {
                    QString name = QString::fromStdString(waypointObj["name"].asString());
                    double latitude = waypointObj["latitude"].asDouble();
                    double longitude = waypointObj["longitude"].asDouble();
                    QString id = QString::fromStdString(waypointObj["id"].asString());

                    this->addWaypointToList(name, latitude, longitude, id);
                    emit addWaypoint(name, latitude, longitude, id);
                }
            }

            RCLCPP_INFO(_node->get_logger(), "Loaded %d waypoints from file", waypointsArray.size());
        }
    }
    else
    {
        RCLCPP_ERROR(_node->get_logger(), "Failed to parse JSON file: %s", errors.c_str());
    }

    file.close();
}
