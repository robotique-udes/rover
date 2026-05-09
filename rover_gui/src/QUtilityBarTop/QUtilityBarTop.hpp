#ifndef Q_UTILITY_BAR_BOTTON_Q_UTILITY_BAR_TOP_HPP
#define Q_UTILITY_BAR_BOTTON_Q_UTILITY_BAR_TOP_HPP

#include "UI_TopUtilityBar.h"

#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/gps.hpp>
#include <rover_msgs/msg/battery.hpp>
#include <rover_msgs/msg/gps.hpp>
#include <rover_msgs/msg/antenna_status.hpp>

#include <rover_lib2/helpers/ip_pinging.hpp>
#include <rover_lib2/helpers/watchdog.hpp>

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <QDateTime>
#include <QTimeZone>

#include <cstddef>

class QUtilityBarTop : public QWidget
{
    Q_OBJECT

    static constexpr const char* TOPIC_BATTERY = "/rover/auxiliary/battery";
    static constexpr const char* TOPIC_ANTENNA_STATUS = "/rover/antenna/status";
    static constexpr const char* TOPIC_GNSS = "/rover/gps/position";

    static constexpr const size_t DELAY_CHECK_BATTERY_PUB_COUNT_MS = 1000UL;
    static constexpr const size_t DELAY_CHECK_RSSI_PUB_COUNT_MS = 1000UL;
    static constexpr const size_t DELAY_CHECK_GNSS_PUB_COUNT_MS = 1000UL;

    static constexpr const size_t DELAY_UPDATE_TIMER_MS = 1000UL;

    static constexpr const size_t WATCH_DOG_DELAY_MS = 1000UL;
    static constexpr const float WATCH_DOG_TIMEOUT = 1.0F;

    static constexpr const char* FILE_PATH = "TaskDateAndTime.txt";

  public:
    QUtilityBarTop(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);

  signals:
    void updateBatteryUI(float _percent);
    void updateAntennaUI(bool connected_, float rssi_, float upSpeed_, float downSpeed_);
    void updateGNSS(uint8_t fix_, float heading_, uint8_t satNbr_, float long_, float lat_);
    void updateTimer(int secondsBeforeTimeOut_);

    void batteryPubCount();
    void antennaStatusPubCount();
    void GNSSPubCount();

    void batteryTimeout();
    void antennaStatusTimeout();
    void GNSSTimeout();

    void timerDisplay(void);

  private slots:
    void onUpdateBatteryUI(float _percent);
    void onUpdateAntennaUI(bool connected_, float rssi_, float upSpeed_, float downSpeed_);
    void onUpdateGNSS(uint8_t fix_, float heading_, uint8_t satNbr_, float long_, float lat_);
    void onUpdateTimer(int secondsBeforeTimeOut_);

    void onBatteryPubCount();
    void onAntennaStatusPubCount();
    void onGNSSPubCount();

    void onBatteryTimeout();
    void onAntennaStatusTimeout();
    void onGNSSTimeout();

    void onTimerDisplay(void);

  private:
    void setupUI(void);

    void initBatterySubscriber(void);
    void initAntennaStatus(void);
    void initGNSS(void);
    void initTimerDisplay(void);
    void initWatchdog(void);

    void readTimersFromFile(const char* filename_, std::vector<QDateTime>& timersList_);

    std::vector<QDateTime> _timersList;
    QTimeZone _timeZone;

    std::shared_ptr<rclcpp::Subscription<rover_msgs::msg::Battery>> _sub_battery;
    std::shared_ptr<rclcpp::Subscription<rover_msgs::msg::AntennaStatus>> _sub_antennaStatus;
    std::shared_ptr<rclcpp::Subscription<rover_msgs::msg::Gps>> _sub_GNSS;

    rclcpp::TimerBase::SharedPtr _timer_batteryPub;
    rclcpp::TimerBase::SharedPtr _timer_RSSIPub;
    rclcpp::TimerBase::SharedPtr _timer_GNSSPub;
    rclcpp::TimerBase::SharedPtr _timer_updateTimer;

    rclcpp::TimerBase::SharedPtr _watchdog_battery;
    rclcpp::TimerBase::SharedPtr _watchdog_GNSS;
    rclcpp::TimerBase::SharedPtr _watchdog_antenna;

    rclcpp::Time _lastGNSSTimeMsg;
    rclcpp::Time _lastBatteryTimeMsg;
    rclcpp::Time _lastAntennaTimeMsg;

    std::shared_ptr<rclcpp::Node> _node;

    rclcpp::Duration _batteryTimeout;
    rclcpp::Duration _GNSSTimeout;
    rclcpp::Duration _antennaTimeout;

    Ui::TopUtilityBar _ui;
};

#endif  // QTOP_UTILITY_BAR
