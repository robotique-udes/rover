#include "QTopUtilityBar.hpp"
#include <cstddef>
#include <cstdint>
#include <qdatetime.h>
#include <qglobal.h>
#include <qtimezone.h>
#include <qicon.h>
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
    this->initGNSS();
    this->initTimerDisplay();

    connect(this, &QTopUtilityBar::updateBatteryUI, this, &QTopUtilityBar::onUpdateBatteryUI);
    connect(this, &QTopUtilityBar::updateWifiUI, this, &QTopUtilityBar::onUpdateWifiUI);
    connect(this, &QTopUtilityBar::updateGNSS, this, &QTopUtilityBar::onUpdateGNSS);
    #warning timer update in ROS and should not

}

void QTopUtilityBar::setupUI(void)
{
    _timeZone = QTimeZone("America/Montreal");

    _ui.batteryLabel->setText("-- %");

    _ui.signalQualityLabel->setText("RSSI: ---   ");
    _ui.connectionSpeedLabel->setText("--.- Mb/s   ");

    _ui.satellitesNbrLabel->setText("Sat: -- ");
    _ui.GNSSFixLabel->setText("Fix: --.-------");
    _ui.HeadingLabel->setText("---.--");

    this->setStyleSheet(R"(
    QPushButton {
        background-color: transparent;
        border: none;
    }
    )");

    #warning heading

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

        _timer_batteryPub = _node->create_wall_timer(std::chrono::milliseconds(DELAY_CHECK_BATTERY_PUB_COUNT_MS),
        [this](void)
        {
            size_t count = _node->count_publishers(TOPIC_BATTERY);
            if(!count)
            {
                QIcon icon(":/icons/BatteryError.svg");
                _ui.batteryIcon->setIcon(icon);
                #warning replace icon with pixmap....
            }
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
        
        _timer_RSSIPub = _node->create_wall_timer(std::chrono::milliseconds(DELAY_CHECK_RSSI_PUB_COUNT_MS),
        [this](void)
        {
            size_t count = _node->count_publishers(TOPIC_WIFI_CONNECTION);
            if(!count)
            {
                QIcon icon(":/icons/ErrorRSSI.png");
                _ui.RSSILabel->setIcon(icon);
            }
        });
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Error, GUI node is invalid");
    }
}

void QTopUtilityBar::initGNSS(void)
{
    if (_node)
    {
        _sub_GNSS = _node->create_subscription<rover_msgs::msg::Gps>(TOPIC_GNSS,
                                                                                 5,
                                                                                 [this](const rover_msgs::msg::Gps msg)
                                                                                 {
                                                                                     CB_GNSS(msg);
                                                                                 });

        _timer_GNSSPub = _node->create_wall_timer(std::chrono::milliseconds(DELAY_CHECK_GNSS_PUB_COUNT_MS),
        [this](void)
        {
            size_t count = _node->count_publishers(TOPIC_GNSS);
            if(!count)
            {
                QIcon iconSat(":/icons/GNSSError.svg");
                QIcon iconHeading(":/icons/HeadingError.svg");

                _ui.satellliteIcon_pb->setIcon(iconSat);
                _ui.headingIcon_pb->setIcon(iconHeading);

                #warning replace icon with pixmap....
            }
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
        QIcon icon(":/icons/BatteryError.svg");
        _ui.batteryIcon->setIcon(icon);
        QHelper::QToastNotification::getInstance().notifyFromAnyThread("Error with battery publisher", "battery publisher is unavailble, please check connection", QHelper::QToastNotification::eNotifType::ERROR);
    }
    else
    {
        //RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Battery percentage: %u", msg_.pourcentage);
        emit this->updateBatteryUI(msg_.pourcentage);
    }
}

void QTopUtilityBar::CB_wifiConnection(rover_msgs::msg::WifiConnection msg_)
{
    bool valid = msg_.valid;
    if(!valid)
    {
        QIcon icon(":/icons/RSSIError.svg");
        _ui.RSSILabel->setIcon(icon);
        QHelper::QToastNotification::getInstance().notifyFromAnyThread("Error with wifi connection publisher", "wifi connection publisher is unavailble, please check connection", QHelper::QToastNotification::eNotifType::ERROR);
    }
    else
    {
        emit this->updateWifiUI(msg_.rssi, msg_.speed_connection);
    }
}

void QTopUtilityBar::CB_GNSS(rover_msgs::msg::Gps msg_)
{
    emit this->updateGNSS(msg_.longitude, msg_.heading,  msg_.satellite);
    #warning fixx not speed!!
}


void QTopUtilityBar::CB_timerDisplaying(void)
{
    int secondsBeforeTimeout = -1;

    for(const auto& timers:_timersList)
    {
        QDateTime givenTimeUTC = timers.toUTC();
        QDateTime nowUtc = QDateTime::currentDateTimeUtc().toTimeZone(_timeZone);

        int secondsDiff = nowUtc.secsTo(givenTimeUTC);

        if((secondsDiff<secondsBeforeTimeout && secondsDiff>0) || (secondsBeforeTimeout == -1 && secondsDiff>0))
        {
            secondsBeforeTimeout = secondsDiff;
        }
    }

    if(secondsBeforeTimeout>=0)
    {
        int hours = secondsBeforeTimeout / 3600;
        int minutes = (secondsBeforeTimeout % 3600) / 60;
        int seconds = secondsBeforeTimeout % 60;
    
        QString timeString = QString("%1:%2:%3")
        .arg(hours, 2, 10, QChar('0'))
        .arg(minutes, 2, 10, QChar('0'))
        .arg(seconds, 2, 10, QChar('0'));

        QDateTime now = QDateTime::currentDateTimeUtc().toTimeZone(_timeZone);

        _ui.timerLabel->setText(timeString);
        if(secondsBeforeTimeout<300)
        {
            _ui.timerLabel->setStyleSheet("QLabel { color : red; }");
        }
        else if(secondsBeforeTimeout<600)
        {
            _ui.timerLabel->setStyleSheet("QLabel { color : orange; }");
        }
        else
        {
            _ui.timerLabel->setStyleSheet("QLabel { color : white; }"); 
        }
    }
    else
    {
    _ui.timerLabel->setText("OVER");
    _ui.timerLabel->setStyleSheet("QLabel { color : red; }");
    }


}

void QTopUtilityBar::onUpdateBatteryUI(uint8_t pourcent_)
{
    _ui.batteryLabel->setText(QString::number(static_cast<int>(pourcent_))+" %");
    QIcon icon;

    if(pourcent_>=85)
    {
        icon = QIcon(":/icons/BatteryIcon100.svg");
    }
    else if(pourcent_>=55)
    {
        icon = QIcon(":/icons/BatteryIcon75.svg");
    }
    else if(pourcent_>=40)
    {
        icon = QIcon(":/icons/BatteryIcon50.svg");
    }
    else if(pourcent_>=20)
    {
        icon = QIcon(":/icons/BatteryIcon25.svg");
    }
    else
    {
        icon = QIcon(":/icons/BatteryIcon0.svg");
    }

    _ui.batteryIcon->setIcon(icon);
}

void QTopUtilityBar::onUpdateWifiUI(float rssi_, float speed_)
{
    _ui.signalQualityLabel->setText("RSSI: " + QString::number(static_cast<int>(rssi_))+ "   ");
    _ui.connectionSpeedLabel->setText(QString::number(static_cast<float>(speed_), 'f',1)+" Mb/s   ");

    QIcon icon;

    if(rssi_<=-85)
    {
        icon = QIcon(":/icons/RSSI_one.png");
    } 
    else if(rssi_>-85 && rssi_<=-75)
    {
        icon = QIcon(":/icons/RSSI_two.png");
    }
    else if(rssi_>-75 && rssi_<=-65)
    {
        qDebug("yess");
        icon = QIcon(":/icons/RSSI_three.png");
    }
    else
    {
        icon = QIcon(":/icons/RSSI_four.png");
    }

    _ui.RSSILabel->setIcon(icon);
    this->update();  // Make sure button repaints immediatelyint();
}

void QTopUtilityBar::onUpdateGNSS(float fix_, float heading_, uint8_t satNbr_)
{
    _ui.HeadingLabel->setText(QString::number(static_cast<float>(heading_), 'f',2)+ " deg   ");
    _ui.satellitesNbrLabel->setText(QString::number(static_cast<int>(satNbr_))+"   ");
    _ui.GNSSFixLabel->setText("Fix: " + QString::number(static_cast<float>(fix_), 'f',6)+"   ");

    _ui.satellliteIcon_pb->setIcon(QIcon(":/icons/GNSSIcon.svg"));
    _ui.headingIcon_pb->setIcon(QIcon(":/icons/HeadingIcon.svg"));
    this->repaint();
}

void QTopUtilityBar::simulateTimerFileReading()
{
    _timersList.push_back(QDateTime(QDate(2025,5,16),QTime(13,58,0),QTimeZone("America/Montreal")));
    _timersList.push_back(QDateTime(QDate(2025,5,16),QTime(18,36,0),QTimeZone("America/Montreal")));
    _timersList.push_back(QDateTime(QDate(2025,5,18),QTime(16,3,0),QTimeZone("America/Edmonton")));
}