#include "QExample.hpp"

void QExample::gpsCallback(const rover_msgs::msg::Gps::SharedPtr rosMsg_)
{
    _ui.lb_latitude->setText(QString::number(rosMsg_->latitude));
    _ui.lb_longitude->setText(QString::number(rosMsg_->longitude));
}