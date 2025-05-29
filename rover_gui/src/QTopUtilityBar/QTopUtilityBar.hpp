#ifndef QTOP_UTILITY_BAR
#define QTOP_UTILITY_BAR

#include "rclcpp/rclcpp.hpp"
#include "rover_lib2/helpers/ip_pinging.hpp"
#include "rover_msgs/msg/gps.hpp"
#include "rover_lib2/helpers/watchdog.hpp"

#include "rclcpp/duration.hpp"
#include "UI_TopUtilityBar.h"
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <cstddef>
#include <rover_msgs/msg/battery.hpp>
#include <rover_msgs/msg/gps.hpp>
#include <rover_msgs/msg/wifi_connection.hpp>
#include <QDateTime>
#include <QTimeZone>

class QTopUtilityBar : public QWidget
{
    Q_OBJECT

    static constexpr const char* TOPIC_BATTERY = "/rover/auxiliary/battery";
    static constexpr const char* TOPIC_WIFI_CONNECTION = "/rover/auxiliary/connection_speed";
    static constexpr const char* TOPIC_GNSS = "/rover/gps/position";

    static constexpr const size_t DELAY_CHECK_BATTERY_PUB_COUNT_MS = 1000UL;
    static constexpr const size_t DELAY_CHECK_RSSI_PUB_COUNT_MS = 1000UL;
    static constexpr const size_t DELAY_CHECK_GNSS_PUB_COUNT_MS = 1000UL;

    static constexpr const size_t DELAY_UPDATE_TIMER_MS = 1000UL;

    static constexpr const size_t WATCH_DOG_DELAY_MS = 1000UL;
    static constexpr const float WATCH_DOG_TIMEOUT = 1.0F;

    static constexpr const char* FILE_PATH = "TaskDateAndTime.txt";

  public:
    QTopUtilityBar(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);

  signals:
    void updateBatteryUI(float _percent);
    void updateWifiUI(float rssi_, float speed_);
    void updateGNSS(uint8_t fix_, float heading_, uint8_t satNbr_, float long_, float lat_);
    void updateTimer(int secondsBeforeTimeOut_);

  private slots:
    void onUpdateBatteryUI(float _percent);
    void onUpdateWifiUI(float rssi_, float speed_);
    void onUpdateGNSS(uint8_t fix_, float heading_, uint8_t satNbr_, float long_, float lat_);
    void onUpdateTimer(int secondsBeforeTimeOut_);

  private:
    void setupUI(void);

    void initBatterySubscriber(void);
    void initWifiConnection(void);
    void initGNSS(void);
    void initTimerDisplay(void);
    void initWatchdog(void);

    void CB_battery(rover_msgs::msg::Battery& msg_);
    void CB_wifiConnection(rover_msgs::msg::WifiConnection& msg_);
    void CB_GNSS(rover_msgs::msg::Gps& msg_);
    void CB_timerDisplaying(void);
    void updateTimeZone(void);

    void CB_batteryPubCount();
    void CB_wifiConnectionPubCount();
    void CB_GNSSPubCount();

    void CB_batteryTimeout();
    void CB_wifiConnectionTimeout();
    void CB_GNSSTimeout();

    void readTimersFromFile(const char* filename_, std::vector<QDateTime> timersList_);

    std::vector<QDateTime> _timersList;
    QTimeZone _timeZone;

    std::shared_ptr<rclcpp::Subscription<rover_msgs::msg::Battery>> _sub_battery;
    std::shared_ptr<rclcpp::Subscription<rover_msgs::msg::WifiConnection>> _sub_wifiConnection;
    std::shared_ptr<rclcpp::Subscription<rover_msgs::msg::Gps>> _sub_GNSS;

    rclcpp::TimerBase::SharedPtr _timer_batteryPub;
    rclcpp::TimerBase::SharedPtr _timer_RSSIPub;
    rclcpp::TimerBase::SharedPtr _timer_GNSSPub;
    rclcpp::TimerBase::SharedPtr _timer_updateTimer;

    rclcpp::TimerBase::SharedPtr _watchdog_battery;
    rclcpp::TimerBase::SharedPtr _watchdog_GNSS;
    rclcpp::TimerBase::SharedPtr _watchdog_wifi;

    rclcpp::Time _lastGNSSTimeMsg;
    rclcpp::Time _lastBatteryTimeMsg;
    rclcpp::Time _lastWifiTimeMsg;

    std::shared_ptr<rclcpp::Node> _node;

    rclcpp::Duration _batteryTimeout;
    rclcpp::Duration _GNSSTimeout;
    rclcpp::Duration _wifiTimeout;

    Ui::TopUtilityBar _ui;
};

#endif  // QTOP_UTILITY_BAR