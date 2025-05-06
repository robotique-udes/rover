#include "QNavigation.hpp"

QNavigation::QNavigation(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    _ui = new Ui::Navigation();
    _ui->setupUi(this);

    _ui->webViewContainer->load(QUrl("qrc:/map.html"));
    _ui->webViewContainer->setMinimumSize(1200, 1000);
    _ui->webViewContainer->setMaximumSize(1200, 1000);

    _webChannel = new QWebChannel(this);
    _webChannel->registerObject(QStringLiteral("bridge"), this);
    _ui->webViewContainer->page()->setWebChannel(_webChannel);

    _gpsSub = _node->create_subscription<rover_msgs::msg::Gps>("/rover/gps/position",
                                                               1,
                                                               [this](rover_msgs::msg::Gps::SharedPtr msg_)
                                                               {
                                                                   _currentLat = msg_->latitude;
                                                                   _currentLon = msg_->longitude;
                                                                   _currentHeading = msg_->heading;

                                                                   QMetaObject::invokeMethod(this,
                                                                                             "gpsCallback",
                                                                                             Qt::QueuedConnection,
                                                                                             Q_ARG(double, msg_->latitude),
                                                                                             Q_ARG(double, msg_->longitude),
                                                                                             Q_ARG(double, msg_->heading));
                                                               });

    connect(_ui->setGoalButton,
            &QPushButton::clicked,
            this,
            [this]()
            {
                if (_ui->inputName->text().isEmpty() || _ui->inputLatitude->text().isEmpty()
                    || _ui->inputLongitude->text().isEmpty())
                {
                    QMessageBox::warning(this, "Input Error", "Please enter waypoint name and coordinates.");
                    return;
                }

                double lat_ = _ui->inputLatitude->text().toDouble();
                double lon_ = _ui->inputLongitude->text().toDouble();
                QString name_ = _ui->inputName->text();
                QString id_ = "waypoint_" + QUuid::createUuid().toString(QUuid::WithoutBraces);

                addWaypointToList(name_, lat_, lon_, id_);

                emit sendGoal(name_, lat_, lon_);

                _ui->inputName->clear();
                _ui->inputLatitude->clear();
                _ui->inputLongitude->clear();
            });

    connect(_ui->calculatePathButton, &QPushButton::clicked, this, &QNavigation::onCalculatePathClicked);
    connect(_ui->waypointList, &QListWidget::itemClicked, this, &QNavigation::onWaypointSelected);
    connect(_ui->clearWaypointsButton, &QPushButton::clicked, this, &QNavigation::onClearWaypointsClicked);
    connect(_ui->clearPathButton, &QPushButton::clicked, this, &QNavigation::onClearPathClicked);
    connect(_ui->deleteWaypointButton, &QPushButton::clicked, this, &QNavigation::onDeleteWaypointClicked);
}

QNavigation::~QNavigation()
{
    delete _ui;
}

void QNavigation::pathDistanceCalculated(double distanceMeters_)
{
    QString distanceText_;
    if (distanceMeters_ >= 1000)
    {
        distanceText_ = QString("%1 km").arg(distanceMeters_ / 1000.0, 0, 'f', 2);
    }
    else
    {
        distanceText_ = QString("%1 m").arg(qRound(distanceMeters_));
    }

    _ui->distanceLabel->setText(distanceText_);
}

void QNavigation::waypointCreated(QString name_, double latitude_, double longitude_, QString id_)
{
    if (id_.isEmpty())
    {
        id_ = "waypoint_" + QUuid::createUuid().toString(QUuid::WithoutBraces);
    }

    addWaypointToList(name_, latitude_, longitude_, id_);
}

void QNavigation::onCalculatePathClicked()
{
    QListWidgetItem* currentItem_ = _ui->waypointList->currentItem();
    if (!currentItem_)
    {
        QMessageBox::information(this, "Select Waypoint", "Please select a waypoint from the list first.");
        return;
    }

    int index_ = _ui->waypointList->row(currentItem_);
    if (index_ >= 0 && index_ < _waypoints.size())
    {
        const Waypoint& waypoint_ = _waypoints.at(index_);

        emit calculatePath(waypoint_.latitude, waypoint_.longitude);
    }
}

void QNavigation::addWaypointToList(const QString& name_, double latitude_, double longitude_, const QString& id_)
{
    Waypoint waypoint_;
    waypoint_.name = name_;
    waypoint_.latitude = latitude_;
    waypoint_.longitude = longitude_;
    waypoint_.id = id_;

    _waypoints.append(waypoint_);

    QString displayText_ = QString("%1 (%2, %3)").arg(name_).arg(latitude_, 0, 'f', 6).arg(longitude_, 0, 'f', 6);
    _ui->waypointList->addItem(displayText_);
}

void QNavigation::onWaypointSelected(QListWidgetItem* item_)
{
    int index_ = _ui->waypointList->row(item_);
    if (index_ >= 0 && index_ < _waypoints.size())
    {
        const Waypoint& waypoint_ = _waypoints.at(index_);

        _ui->inputName->setText(waypoint_.name);
        _ui->inputLatitude->setText(QString::number(waypoint_.latitude, 'f', 6));
        _ui->inputLongitude->setText(QString::number(waypoint_.longitude, 'f', 6));
    }
}

void QNavigation::onDeleteWaypointClicked()
{
    QListWidgetItem* currentItem_ = _ui->waypointList->currentItem();
    if (!currentItem_)
    {
        QMessageBox::information(this, "Select Waypoint", "Please select a waypoint to delete.");
        return;
    }

    int index_ = _ui->waypointList->row(currentItem_);
    if (index_ >= 0 && index_ < _waypoints.size())
    {
        const QString waypointId_ = _waypoints.at(index_).id;

        delete _ui->waypointList->takeItem(index_);

        emit deleteWaypoint(waypointId_);

        _waypoints.removeAt(index_);

        _ui->inputName->clear();
        _ui->inputLatitude->clear();
        _ui->inputLongitude->clear();

        _ui->distanceLabel->setText("N/A");

        emit clearPath();
    }
}

void QNavigation::onClearWaypointsClicked()
{
    int result_ = QMessageBox::question(this,
                                        "Clear Waypoints",
                                        "Are you sure you want to clear all waypoints?",
                                        QMessageBox::Yes | QMessageBox::No);

    if (result_ == QMessageBox::Yes)
    {
        _waypoints.clear();
        _ui->waypointList->clear();

        _ui->inputName->clear();
        _ui->inputLatitude->clear();
        _ui->inputLongitude->clear();
        _ui->distanceLabel->setText("N/A");

        qDebug() << "Emitting clearWaypoints signal";
        emit clearWaypoints();
    }
}

void QNavigation::onClearPathClicked()
{
    _ui->distanceLabel->setText("N/A");

    emit clearPath();
}
