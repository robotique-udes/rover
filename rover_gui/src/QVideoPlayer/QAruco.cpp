#include "QAruco.hpp"

QAruco::QAruco(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    _ui.setupUi(this);

    connect(_ui.pushButton, &QPushButton::clicked, this, [this]() { emit this->startDetection()})
    

    //_sub_gps = _node->create_subscription<rover_msgs::msg::Gps>("/rover/gps/position",
    //                                                            1,
    //                                                           std::bind(&QExample::gpsCallback, this, std::placeholders::_1));
}

/*void QExample::gpsCallback(const rover_msgs::msg::Gps::SharedPtr rosMsg_)
{
    _ui.lb_latitude->setText(QString::number(rosMsg_->latitude));
    _ui.lb_longitude->setText(QString::number(rosMsg_->longitude));
}*/

void QAruco::startDetection(std::string camURL_)
{
    return;
}