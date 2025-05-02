#include "QNavigation.hpp"
#include <QMessageBox>

QNavigation::QNavigation(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    ui = new Ui::Navigation();
    ui->setupUi(this);

    ui->webViewContainer->load(QUrl("qrc:/map.html"));

    ui->webViewContainer->setMinimumSize(1200, 1000);
    ui->webViewContainer->setMaximumSize(1200, 1000);

    webChannel = new QWebChannel(this);
    webChannel->registerObject(QStringLiteral("bridge"), this);
    ui->webViewContainer->page()->setWebChannel(webChannel);

    // Connect subscription to the GPS topic
    _gpsSub = _node->create_subscription<rover_msgs::msg::Gps>("/rover/gps/position",
                                                               1,
                                                               [this](rover_msgs::msg::Gps::SharedPtr msg)
                                                               {
                                                                   _currentLat = msg->latitude;
                                                                   _currentLon = msg->longitude;
                                                                   _currentHeading = msg->heading;
                                                                   
                                                                   QMetaObject::invokeMethod(this,
                                                                                             "gpsCallback",
                                                                                             Qt::QueuedConnection,
                                                                                             Q_ARG(double, msg->latitude),
                                                                                             Q_ARG(double, msg->longitude),
                                                                                             Q_ARG(double, msg->heading));
                                                               });

    // Connect the Set Goal button to emit the sendGoal signal
    connect(ui->setGoalButton,
            &QPushButton::clicked,
            this,
            [this]()
            {
                // Validate input fields
                if (ui->inputName->text().isEmpty() || 
                    ui->inputLatitude->text().isEmpty() || 
                    ui->inputLongitude->text().isEmpty()) {
                    QMessageBox::warning(this, "Input Error", "Please enter waypoint name and coordinates.");
                    return;
                }

                double lat = ui->inputLatitude->text().toDouble();
                double lon = ui->inputLongitude->text().toDouble();
                QString name = ui->inputName->text();
                
                // Add waypoint to the list
                addWaypointToList(name, lat, lon);
                
                // Send the goal to the map
                emit sendGoal(name, lat, lon);
                
                // Clear input fields
                ui->inputName->clear();
                ui->inputLatitude->clear();
                ui->inputLongitude->clear();
            });
            
    // Connect the Calculate Path button
    connect(ui->calculatePathButton,
            &QPushButton::clicked,
            this,
            &QNavigation::onCalculatePathClicked);
            
    // Connect the waypoint selection
    connect(ui->waypointList, 
            &QListWidget::itemClicked, 
            this, 
            &QNavigation::onWaypointSelected);
            
    // Connect the Clear Waypoints button
    connect(ui->clearWaypointsButton,
            &QPushButton::clicked,
            this,
            &QNavigation::onClearWaypointsClicked);
}

QNavigation::~QNavigation()
{
    delete ui;
}

void QNavigation::pathDistanceCalculated(double distanceMeters)
{
    // Format and display the distance
    QString distanceText;
    if (distanceMeters >= 1000) {
        distanceText = QString("%1 km").arg(distanceMeters / 1000.0, 0, 'f', 2);
    } else {
        distanceText = QString("%1 m").arg(qRound(distanceMeters));
    }
    
    ui->distanceLabel->setText(distanceText);
}

void QNavigation::onCalculatePathClicked()
{
    // Check if a waypoint is selected in the list
    QListWidgetItem* currentItem = ui->waypointList->currentItem();
    if (!currentItem) {
        QMessageBox::information(this, "Select Waypoint", "Please select a waypoint from the list first.");
        return;
    }
    
    // Get the index of the selected waypoint
    int index = ui->waypointList->row(currentItem);
    if (index >= 0 && index < _waypoints.size()) {
        // Get the selected waypoint
        const Waypoint& waypoint = _waypoints.at(index);
        
        // Calculate path to this waypoint
        emit calculatePath(waypoint.latitude, waypoint.longitude);
    }
}

void QNavigation::addWaypointToList(const QString& name, double latitude, double longitude)
{
    // Create new waypoint
    Waypoint waypoint;
    waypoint.name = name;
    waypoint.latitude = latitude;
    waypoint.longitude = longitude;
    
    // Add to internal list
    _waypoints.append(waypoint);
    
    // Add to UI list
    QString displayText = QString("%1 (%.6f, %.6f)").arg(name).arg(latitude, 0, 'f', 6).arg(longitude, 0, 'f', 6);
    ui->waypointList->addItem(displayText);
}

void QNavigation::onWaypointSelected(QListWidgetItem* item)
{
    // When a waypoint is selected from the list, populate the input fields with its data
    int index = ui->waypointList->row(item);
    if (index >= 0 && index < _waypoints.size()) {
        const Waypoint& waypoint = _waypoints.at(index);
        
        ui->inputName->setText(waypoint.name);
        ui->inputLatitude->setText(QString::number(waypoint.latitude, 'f', 6));
        ui->inputLongitude->setText(QString::number(waypoint.longitude, 'f', 6));
    }
}

void QNavigation::onClearWaypointsClicked()
{
    // Confirm with user
    int result = QMessageBox::question(this, 
                                      "Clear Waypoints", 
                                      "Are you sure you want to clear all waypoints?",
                                      QMessageBox::Yes | QMessageBox::No);
                                      
    if (result == QMessageBox::Yes) {
        // Clear the waypoint list
        _waypoints.clear();
        ui->waypointList->clear();
        
        // Clear input fields
        ui->inputName->clear();
        ui->inputLatitude->clear();
        ui->inputLongitude->clear();
        ui->distanceLabel->setText("N/A");
    }
}