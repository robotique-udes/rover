#include "QAruco.hpp"

QAruco::QAruco(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{

    _client_ArucoDetectionManager = _node->create_client<rover_msgs::srv::ArucoDetection>(
        "/rover/auxiliary/aruco/manager");

    _ui.setupUi(this);

    connect(_ui.arucoPushButton, &QPushButton::clicked, this, &QAruco::startDetection);

    connect(&_playerWorkerThread, &QPlayerWorker::detectionStartedSuccessfully, this, &QAruco::onDetectionStarted);

    _playerWorkerThread.start();


    //_sub_gps = _node->create_subscription<rover_msgs::msg::Gps>("/rover/gps/position",
    //                                                            1,
    //                                                           std::bind(&QExample::gpsCallback, this, std::placeholders::_1));
}

/*void QExample::gpsCallback(const rover_msgs::msg::Gps::SharedPtr rosMsg_)
{
    _ui.lb_latitude->setText(QString::number(rosMsg_->latitude));
    _ui.lb_longitude->setText(QString::number(rosMsg_->longitude));
}*/

void QAruco::startDetection()
{
    _playerWorkerThread.startDetection(_node,_client_ArucoDetectionManager, "rtsp://127.0.0.1:8554/live");
}

void QAruco::onDetectionStarted(bool success) {
        qDebug() << "result!! " << success;
    }