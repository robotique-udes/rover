#include "QNavigation.hpp"

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
                double lat = ui->inputLatitude->text().toDouble();
                double lon = ui->inputLongitude->text().toDouble();
                emit sendGoal(lat, lon);  // Emit signal here
            });
}

QNavigation::~QNavigation()
{
    delete ui;
}
