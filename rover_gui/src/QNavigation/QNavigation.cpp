#include "QNavigation.hpp"

// QT
#include <QRegularExpression>
#include <QTimer>
#include <cmath>
#include "Global/Helpers/QHelpers.hpp"
#include <Global/Helpers/QToastNotification/QToastNotification.hpp>
#include "Global/Helpers/QSessionFolderManager/QSessionFolderManager.hpp"

// Helpers
#include <rover_lib2/helpers/folders.hpp>

namespace
{
    QString toDMS(double deg, bool isLat)
    {
        char dir;

        if (isLat)
            dir = (deg >= 0) ? 'N' : 'S';
        else
            dir = (deg >= 0) ? 'E' : 'W';

        deg = std::abs(deg);

        int d = static_cast<int>(deg);
        double minFloat = (deg - d) * 60.0;
        int m = static_cast<int>(minFloat);
        double s = (minFloat - m) * 60.0;

        return QString("%1° %2' %3\" %4")
            .arg(d)
            .arg(m)
            .arg(s, 0, 'f', 2)
            .arg(dir);
    }
}

constexpr const char* QRC_PATH_MAP_HTML = "qrc:/other/map.html";
constexpr const char* GPS_TOPIC_NAME = "/rover/gps/position";
constexpr const char* NAVIGATION_PATH = "/Navigation";

constexpr float CSV_WRITE_FREQUENCY_HZ = 1.0F;
// Default to Studio de Création
constexpr double DEFAULT_LATITUDE = 45.377755;
constexpr double DEFAULT_LONGITUDE = -71.924652;
constexpr double DEFAULT_HEADING = 0.0;

QNavigation::QNavigation(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _webChannel(this),
    _node(guiNode_),
    _pathManager(true, this)
{
    _ui.setupUi(this);
    this->createNavigationFolder();
    this->initializeWaypointManager();
    this->initializePathManager();
    this->updateCoordinateLabels();

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
    _csvWriteTimer = _node->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / CSV_WRITE_FREQUENCY_HZ)),
                                              [this]()
                                              {
                                                  this->onCSVWriteTimer();
                                              });
}

void QNavigation::initializeWaypointManager(void)
{
    _waypointManager.setSessionFolderPath(_sessionFolderPath);
    _waypointManager.initializeWaypoints();

    _waypointManager.syncWaypoints(_waypointsList);

    for (const sWaypoint& waypoint : _waypointsList)
    {
        this->addWaypointToUI(waypoint);
    }
}

void QNavigation::initializePathManager(void)
{
    _pathManager.setSessionFolderPath(_sessionFolderPath);
    _pathManager.initializeCSVFile(_oldPath);
}

void QNavigation::onJsBridgeReady(void)
{
    for (const sWaypoint& waypoint : _waypointsList)
    {
        emit this->sendGoal(QString::fromStdString(waypoint.name),
                            waypoint.latitude,
                            waypoint.longitude,
                            QString::fromStdString(waypoint.id),
                            false);
    }
    emit this->gpsCallback(DEFAULT_LATITUDE, DEFAULT_LONGITUDE, DEFAULT_HEADING);

    emit this->loadFullPath(_oldPath);
}

void QNavigation::createNavigationFolder(void)
{
    std::optional<std::string> optionalSessionFolderPath = QSessionFolderManager::getInstance().getSessionFolderPath();
    std::string homePath;
    std::string sessionPath;

    if (optionalSessionFolderPath.has_value())
    {
        sessionPath = optionalSessionFolderPath.value();
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
        homePath = optionalHomePath.value();
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

    _latestGpsMsg = msg_;

    if (!_hasGpsData)
        _hasGpsData = true;
}

void QNavigation::onCSVWriteTimer(void)
{
    if (_hasGpsData)
    {
        emit this->updatePathTaken(_latestGpsMsg.latitude, _latestGpsMsg.longitude);
        _pathManager.writePosToCSV(_latestGpsMsg.latitude, _latestGpsMsg.longitude);
    }
}

void QNavigation::onSetGoalClicked()
{
    if (_ui.inputName->text().isEmpty() || _ui.inputLatitude->text().isEmpty() || _ui.inputLongitude->text().isEmpty())
    {
        QHelper::QPopUp::sendQuestionPopUp("Input Error", "Please enter waypoint name and coordinates.");
        return;
    }

    sWaypoint waypoint;
    bool okLatitude = false;
    bool okLongitude = false;
    waypoint.latitude = this->parseCoordinateText(_ui.inputLatitude->text(), okLatitude);
    waypoint.longitude = this->parseCoordinateText(_ui.inputLongitude->text(), okLongitude);
    waypoint.name = _ui.inputName->text().toStdString();

    if (!okLatitude || !okLongitude)
    {
        QHelper::QPopUp::sendQuestionPopUp("Input Error", "Please enter valid coordinates in DD or DMS format.");
        return;
    }

    for (const sWaypoint& waypointIt : _waypointsList)
    {
        if (waypointIt.name == waypoint.name || waypointIt.latitude == waypoint.latitude
            || waypointIt.longitude == waypoint.longitude)
        {
            QHelper::QPopUp::sendQuestionPopUp(
                "Duplicate Name or duplicate location",
                "A waypoint with this name or position already exists. Please choose a different name or position.");
            return;
        }
    }

    waypoint.id = "waypoint_" + QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();

    this->addWaypointToList(waypoint);
    _waypointManager.syncWaypoints(_waypointsList);
    emit this->sendGoal(QString::fromStdString(waypoint.name),
                        waypoint.latitude,
                        waypoint.longitude,
                        QString::fromStdString(waypoint.id),
                        true);

    _ui.inputName->clear();
    _ui.inputLatitude->clear();
    _ui.inputLongitude->clear();
}

void QNavigation::pathDistanceCalculated(double distanceMeters_, double heading_)
{
    QString distanceText_;
    if (distanceMeters_ >= 1000.0)
    {
        distanceText_ = QString("%1 km, %2 deg").arg(distanceMeters_ / 1000.0, 0, 'f', 2).arg(heading_, 0, 'f', 2);
    }
    else
    {
        distanceText_ = QString("%1 m, %2 deg").arg(qRound(distanceMeters_)).arg(heading_, 0, 'f', 2);
    }

    _ui.distanceLabel->setText(distanceText_);
}

void QNavigation::waypointCreated(const QString& name_, double latitude_, double longitude_, const QString& id_)
{
    for (const auto& waypoint : _waypointsList)
    {
        if (waypoint.id == id_.toStdString() || waypoint.name == name_.toStdString())
        {
            return;
        }
    }

    std::string id;
    id = id_.isEmpty() ? "waypoint_" + QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString() : id_.toStdString();

    sWaypoint waypoint = {name_.toStdString(), latitude_, longitude_, id};
    this->addWaypointToList(waypoint);
    _waypointManager.syncWaypoints(_waypointsList);
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
    if (index_ >= 0 && index_ < _waypointsList.size())
    {
        const sWaypoint& waypoint = _waypointsList.at(index_);

        emit this->calculatePath(waypoint.latitude, waypoint.longitude, QString::fromStdString(waypoint.id));
    }
}

void QNavigation::addWaypointToList(const sWaypoint& waypoint_)
{
    _waypointsList.append(waypoint_);

    bool exists = false;
    for (int i = 0; i < _ui.waypointList->count(); ++i)
    {
        QListWidgetItem* item = _ui.waypointList->item(i);
        if (item->data(Qt::UserRole).toString().toStdString() == waypoint_.id)
        {
            exists = true;
            break;
        }
    }

    if (!exists)
    {
        this->addWaypointToUI(waypoint_);
    }
}

QString QNavigation::waypointDisplayText(const sWaypoint& waypoint_) const
{
    if (_dmsOn)
    {
        return QString("%1 (%2, %3)")
            .arg(QString::fromStdString(waypoint_.name))
            .arg(toDMS(waypoint_.latitude, true))
            .arg(toDMS(waypoint_.longitude, false));
    }

    return QString("%1 (%2, %3)")
        .arg(QString::fromStdString(waypoint_.name))
        .arg(waypoint_.latitude, 0, 'f', 6)
        .arg(waypoint_.longitude, 0, 'f', 6);
}

double QNavigation::parseCoordinateText(const QString& text, bool& ok) const
{
    ok = false;
    QString trimmed = text.trimmed();
    if (trimmed.isEmpty())
    {
        return 0.0;
    }

    static const QRegularExpression regex(R"(^\s*([+-]?\d+(?:\.\d+)?)(?:\s*°\s*([0-9]+(?:\.\d+)?)\s*'\s*([0-9]+(?:\.\d+)?)\s*"?)?\s*([NnSsEeWw])?\s*$)");
    QRegularExpressionMatch match = regex.match(trimmed);
    if (!match.hasMatch())
    {
        bool localOk = false;
        double value = trimmed.toDouble(&localOk);
        ok = localOk;
        return value;
    }

    double degrees = match.captured(1).toDouble(&ok);
    if (!ok)
    {
        return 0.0;
    }

    double minutes = 0.0;
    double seconds = 0.0;
    if (!match.captured(2).isEmpty())
    {
        minutes = match.captured(2).toDouble(&ok);
        if (!ok)
        {
            return 0.0;
        }
    }
    if (!match.captured(3).isEmpty())
    {
        seconds = match.captured(3).toDouble(&ok);
        if (!ok)
        {
            return 0.0;
        }
    }

    double value = std::abs(degrees) + minutes / 60.0 + seconds / 3600.0;
    QString direction = match.captured(4).toUpper();

    if (!direction.isEmpty())
    {
        if (direction == "S" || direction == "W")
        {
            value = -std::abs(value);
        }
        else
        {
            value = std::abs(value);
        }
    }
    else if (degrees < 0)
    {
        value = -value;
    }

    ok = true;
    return value;
}

void QNavigation::updateCoordinateLabels(void)
{
    _ui.labelLatitude->setText(QString("Latitude (%1):").arg(_dmsOn ? "DMS" : "DD"));
    _ui.labelLongitude->setText(QString("Longitude (%1):").arg(_dmsOn ? "DMS" : "DD"));
}

void QNavigation::refreshCoordinateInputs(void)
{
    if (!_ui.inputLatitude->text().isEmpty())
    {
        bool ok = false;
        double latitude = this->parseCoordinateText(_ui.inputLatitude->text(), ok);
        if (ok)
        {
            _ui.inputLatitude->setText(_dmsOn ? toDMS(latitude, true)
                                              : QString::number(latitude, 'f', 6));
        }
    }

    if (!_ui.inputLongitude->text().isEmpty())
    {
        bool ok = false;
        double longitude = this->parseCoordinateText(_ui.inputLongitude->text(), ok);
        if (ok)
        {
            _ui.inputLongitude->setText(_dmsOn ? toDMS(longitude, false)
                                               : QString::number(longitude, 'f', 6));
        }
    }
}

void QNavigation::addWaypointToUI(const sWaypoint& waypoint_, Qt::CheckState checkState)
{
    QString displayText = this->waypointDisplayText(waypoint_);

    std::unique_ptr<QListWidgetItem> waypointItem = std::make_unique<QListWidgetItem>(displayText);

    waypointItem->setFlags(waypointItem->flags() | Qt::ItemIsUserCheckable);
    waypointItem->setCheckState(checkState);
    waypointItem->setData(Qt::UserRole, QString::fromStdString(waypoint_.id));

    _ui.waypointList->addItem(waypointItem.release());
}

void QNavigation::refreshWaypointItems(void)
{
    if (_ui.waypointList->count() != _waypointsList.size())
    {
        _ui.waypointList->clear();
        for (const sWaypoint& waypoint : _waypointsList)
        {
            this->addWaypointToUI(waypoint, Qt::Checked);
        }
        return;
    }

    for (int i = 0; i < _waypointsList.size(); ++i)
    {
        QListWidgetItem* item = _ui.waypointList->item(i);
        if (item)
        {
            item->setText(this->waypointDisplayText(_waypointsList.at(i)));
        }
    }
}

void QNavigation::onWaypointVisibilityChanged(QListWidgetItem* item_)
{
    if (!item_)
    {
        return;
    }

    int index = _ui.waypointList->row(item_);
    if (index >= 0 && index < _waypointsList.size())
    {
        const sWaypoint& waypoint = _waypointsList.at(index);
        bool isVisible = (item_->checkState() == Qt::Checked);

        emit this->waypointIsVisible(QString::fromStdString(waypoint.id), isVisible);
    }
}

void QNavigation::onWaypointSelected(QListWidgetItem* item_)
{
    if (!item_)
    {
        return;
    }

    int index_ = _ui.waypointList->row(item_);
    if (index_ >= 0 && index_ < _waypointsList.size())
    {
        const sWaypoint& waypoint_ = _waypointsList.at(index_);

        _ui.inputName->setText(QString::fromStdString(waypoint_.name));
        _ui.inputLatitude->setText(_dmsOn ? toDMS(waypoint_.latitude, true)
                                          : QString::number(waypoint_.latitude, 'f', 6));
        _ui.inputLongitude->setText(_dmsOn ? toDMS(waypoint_.longitude, false)
                                           : QString::number(waypoint_.longitude, 'f', 6));
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
    if (index_ >= 0 && index_ < _waypointsList.size())
    {
        const std::string waypointId = _waypointsList.at(index_).id;
        _waypointManager.deleteWaypointFromJson(waypointId);

        delete _ui.waypointList->takeItem(index_);

        emit this->deleteWaypoint(QString::fromStdString(waypointId));

        _waypointsList.removeAt(index_);

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
        _waypointsList.clear();
        _ui.waypointList->clear();
        _waypointManager.clearWaypoints();

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
}

void QNavigation::onClearPathClicked(void)
{
    _ui.distanceLabel->setText("N/A");

    emit this->clearPath();
}

void QNavigation::toggleDMS(int activate_)
{
    const bool newDmsOn = (activate_ != 0);
    if (_dmsOn == newDmsOn)
    {
        return;
    }

    _dmsOn = newDmsOn;
    this->updateCoordinateLabels();
    this->refreshWaypointItems();
    this->refreshCoordinateInputs();
}