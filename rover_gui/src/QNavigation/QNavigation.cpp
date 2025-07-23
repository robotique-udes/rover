#include "QNavigation.hpp"

#include "Global/Helpers/QHelpers.hpp"
#include <QTimer>

constexpr const char* QRC_PATH_MAP_HTML = "qrc:/other/map.html";
constexpr const char* GPS_TOPIC_NAME = "/rover/gps/position";

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

    QString token = qgetenv("CESIUM_TOKEN");
    if (!token.isEmpty())
    {
        _ui.webViewContainer->page()->runJavaScript("Cesium.Ion.defaultAccessToken = '" + token + "';");
    }
    else
    {
        RCLCPP_WARN(_node->get_logger(),
                    "Cesium token not found. This access token is generated with the creation of a Ceisum account. Please refer "
                    "to documentation for more detailed information");
    }

    _ui.webViewContainer->load(QUrl(QRC_PATH_MAP_HTML));

    _webChannel.registerObject(QStringLiteral("bridge"), this);
    _ui.webViewContainer->page()->setWebChannel(&_webChannel);

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

    double lat_ = _ui.inputLatitude->text().toDouble();
    double lon_ = _ui.inputLongitude->text().toDouble();
    QString name_ = _ui.inputName->text();

    for (const auto& waypoint : _waypoints)
    {
        if (waypoint.name == name_)
        {
            QHelper::QPopUp::sendQuestionPopUp("Duplicate Name",
                                               "A waypoint with this name already exists. Please choose a different name.");
            return;
        }
    }

    QString id_ = "waypoint_" + QUuid::createUuid().toString(QUuid::WithoutBraces);

    this->addWaypointToList(name_, lat_, lon_, id_);

    emit this->sendGoal(name_, lat_, lon_);

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
        const Waypoint& waypoint_ = _waypoints.at(index_);

        emit this->calculatePath(waypoint_.latitude, waypoint_.longitude, waypoint_.id);
    }
}

void QNavigation::addWaypointToList(const QString& name_, double latitude_, double longitude_, const QString& id_)
{
    Waypoint waypoint_;
    waypoint_.name = name_;
    waypoint_.latitude = latitude_;
    waypoint_.longitude = longitude_;
    waypoint_.id = id_;

    QString displayText_ = QString("%1 (%2, %3)").arg(name_).arg(latitude_, 0, 'f', 6).arg(longitude_, 0, 'f', 6);

    QListWidgetItem* waypointItem_ = new QListWidgetItem(displayText_);

    waypointItem_->setFlags(waypointItem_->flags() | Qt::ItemIsUserCheckable);
    waypointItem_->setCheckState(Qt::Checked);
    waypointItem_->setData(Qt::UserRole, id_);

    _waypoints.append(waypoint_);

    _ui.waypointList->addItem(waypointItem_);

    onWaypointVisibilityChanged(waypointItem_);
}

void QNavigation::onWaypointVisibilityChanged(QListWidgetItem* item_)
{
    if (!item_)
    {
        return;
    }
    
    int index_ = _ui.waypointList->row(item_);
    if (index_ >= 0 && index_ < _waypoints.size())
    {
        const Waypoint& waypoint_ = _waypoints.at(index_);
        bool isVisible = (item_->checkState() == Qt::Checked);

        emit this->waypointIsVisible(waypoint_.latitude, waypoint_.longitude, waypoint_.name, waypoint_.id, isVisible);
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
        const Waypoint& waypoint_ = _waypoints.at(index_);

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
