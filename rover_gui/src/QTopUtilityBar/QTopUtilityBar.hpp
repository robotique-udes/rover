#ifndef QTOP_UTILITY_BAR
#define QTOP_UTILITY_BAR

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/gps.hpp"

#include "UI_TopUtilityBar.h"
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <cstdint>
#include <rover_msgs/msg/battery.hpp>
#include <rover_msgs/msg/detail/wifi_connection__struct.hpp>
#include <QDateTime>
#include <QTimeZone>



class QTopUtilityBar : public QWidget
{
    Q_OBJECT

    static constexpr const char* TOPIC_BATTERY = "/rover/auxiliary/battery";
    static constexpr const char* TOPIC_WIFI_CONNECTION = "/rover/auxiliary/connection_speed";
    static constexpr const size_t DELAY_CHECK_BATTERY_PUB_COUNT_MS = 1000UL;
    static constexpr const size_t DELAY_CHECK_RSSI_PUB_COUNT_MS = 1000UL;
    static constexpr const size_t DELAY_UPDATE_TIMER_MS = 1000UL;


  public:
  
    QTopUtilityBar(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);

  signals:
    void updateBatteryUI(uint8_t pourcent_);
    void updateWifiUI(float rssi_, float speed_);
    
  private slots:
    void onUpdateBatteryUI(uint8_t pourcent_);
    void onUpdateWifiUI(float rssi_, float speed_);
    
  private:

    void setupUI(void);

    void initBatterySubscriber(void);
    void initWifiConnection(void);
    void initTimerDisplay(void);

    void CB_battery(rover_msgs::msg::Battery msg_);
    void CB_wifiConnection(rover_msgs::msg::WifiConnection msg_);
    void CB_timerDisplaying(void);
    void updateTimeZone(void);
   



    void simulateTimerFileReading(void);


    bool _batteryInvalidFlag = false;
    bool _wifiConnectionFlag = false;

    std::vector<QDateTime> _timersList;
    QTimeZone _timeZone;

    std::shared_ptr<rclcpp::Subscription<rover_msgs::msg::Battery>> _sub_battery;
    std::shared_ptr<rclcpp::Subscription<rover_msgs::msg::WifiConnection>> _sub_wifiConnection;
    rclcpp::TimerBase::SharedPtr _timer_batteryPub;
    rclcpp::TimerBase::SharedPtr _timer_RSSIPub;
    rclcpp::TimerBase::SharedPtr _timer_updateTimer;

    std::shared_ptr<rclcpp::Node> _node;

    Ui::TopUtilityBar _ui;



        
};


#endif //QTOP_UTILITY_BAR