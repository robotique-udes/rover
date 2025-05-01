#include "QNavigation.hpp"

QNavigation::QNavigation(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    ui = new Ui::Navigation();
    ui->setupUi(this);

    ui->webViewContainer->load(QUrl("qrc:/map.html"));

    webChannel = new QWebChannel(this);
    webChannel->registerObject(QStringLiteral("bridge"), this);
    ui->webViewContainer->page()->setWebChannel(webChannel);

    _gpsSub = _node->create_subscription<rover_msgs::msg::Gps>("/rover/gps/position",
                                                               1,
                                                               [this](rover_msgs::msg::Gps::SharedPtr msg)
                                                               {
                                                                   QMetaObject::invokeMethod(this,
                                                                                             "updatePosition",
                                                                                             Qt::QueuedConnection,
                                                                                             Q_ARG(double, msg->latitude),
                                                                                             Q_ARG(double, msg->longitude),
                                                                                             Q_ARG(double, msg->heading));
                                                               });
}

QNavigation::~QNavigation()
{
    delete ui;
}
