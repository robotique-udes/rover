#include "QTopUtilityBar.hpp"
#include <qdatetime.h>
#include <qglobal.h>
#include <qtimezone.h>
#include <rover_msgs/msg/battery.hpp>
#include <rover_msgs/msg/detail/wifi_connection__struct.hpp>
#include <rover_msgs/msg/wifi_connection.hpp>
#include "Global/Helpers/QToastNotification/QToastNotification.hpp"

QTopUtilityBar::QTopUtilityBar(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_)
{
    _ui.setupUi(this);
    this->setupUI();
    this->initBatterySubscriber();
    this->initWifiConnection();
    this->initTimerDisplay();

    connect(_ui.timeZone_pb, &QPushButton::clicked, this, &QTopUtilityBar::updateTimeZone);


}

void QTopUtilityBar::setupUI(void)
{
    _timeZone = QTimeZone("America/Montreal");
    _ui.timeZone_pb->setText("QC");

}

void QTopUtilityBar::initBatterySubscriber(void)
{
    if (_node)
    {
        _sub_battery = _node->create_subscription<rover_msgs::msg::Battery>(TOPIC_BATTERY,
                                                                                 5,
                                                                                 [this](const rover_msgs::msg::Battery msg)
                                                                                 {
                                                                                     CB_battery(msg);
                                                                                 });
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, GUI node is invalid");
    }
}

void QTopUtilityBar::initWifiConnection(void)
{
    if (_node)
    {
        _sub_wifiConnection = _node->create_subscription<rover_msgs::msg::WifiConnection>(TOPIC_WIFI_CONNECTION,
                                                                                 5,
                                                                                 [this](const rover_msgs::msg::WifiConnection msg)
                                                                                 {
                                                                                     CB_wifiConnection(msg);
                                                                                 });
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, GUI node is invalid");
    }
}

void QTopUtilityBar::initTimerDisplay(void)
{
    this->simulateTimerFileReading();
    _timer_updateTimer = _node->create_wall_timer(std::chrono::milliseconds(DELAY_UPDATE_TIMER_MS),
    [this](void)
    {
        this->CB_timerDisplaying();
    });
}

void QTopUtilityBar::CB_battery(rover_msgs::msg::Battery msg_)
{
    bool valid = msg_.valid;
    if(!valid)
    {
        #warning counter to flag after 10 false
        QHelper::QToastNotification::getInstance().notifyFromAnyThread("Error with battery publisher", "battery publisher is unavailble, please check connection", QHelper::QToastNotification::eNotifType::ERROR);
    }
    else
    {
        //RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Battery percentage: %u", msg_.pourcentage);
        _ui.batteryLabel->setText(QString::number(static_cast<int>(msg_.pourcentage))+" %");
    }
}

void QTopUtilityBar::CB_wifiConnection(rover_msgs::msg::WifiConnection msg_)
{
    bool valid = msg_.valid;
    if(!valid)
    {
        #warning counter to flag after 10 false
        QHelper::QToastNotification::getInstance().notifyFromAnyThread("Error with wifi connection publisher", "wifi connection publisher is unavailble, please check connection", QHelper::QToastNotification::eNotifType::ERROR);
    }
    else
    {
        _ui.signalQualityLabel->setText("RSSI: " + QString::number(static_cast<int>(msg_.rssi))+ "   ");
        _ui.connectionSpeedLabel->setText(QString::number(static_cast<float>(msg_.speed_connection), 'f',1)+" Mb/s   ");
    }

}

void QTopUtilityBar::CB_timerDisplaying(void)
{
    QDateTime givenTimeUTC = _timersList.at(0).toUTC();
    qDebug() << _timersList[0].timeZone().id();
    QDateTime nowUtc = QDateTime::currentDateTimeUtc().toTimeZone(_timeZone);

    qint64 secondsDiff = nowUtc.secsTo(givenTimeUTC);

    if(secondsDiff>=0)
    {
        int hours = secondsDiff / 3600;
        int minutes = (secondsDiff % 3600) / 60;
        int seconds = secondsDiff % 60;
    

    QString timeString = QString("%1:%2:%3")
    .arg(hours, 2, 10, QChar('0'))
    .arg(minutes, 2, 10, QChar('0'))
    .arg(seconds, 2, 10, QChar('0'));

    QDateTime now = QDateTime::currentDateTimeUtc().toTimeZone(_timeZone);
    qDebug() << "Current Time (" << _timeZone.id() << "): " << now.toString("yyyy-MM-dd hh:mm:ss");

    _ui.timerLabel->setText(timeString);
    } 
    else
    {
    _ui.timerLabel->setText("OVER");
    }
}

void QTopUtilityBar::updateTimeZone(void)
{
    if(_ui.timeZone_pb->text() == "QC")
    {
        _timeZone = QTimeZone("America/Edmonton");
        _ui.timeZone_pb->setText("AB");
    }
    else
    {
        _timeZone = QTimeZone("America/Montreal");
        _ui.timeZone_pb->setText("QC");
    }
}


void QTopUtilityBar::simulateTimerFileReading()
{
    _timersList.push_back(QDateTime(QDate(2025,5,11),QTime(14,30,0)));
    _timersList.push_back(QDateTime(QDate(2025,5,11),QTime(16,30,0)));
    _timersList.push_back(QDateTime(QDate(2025,5,11),QTime(18,30,0)));
}