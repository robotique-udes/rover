#ifndef QTOP_UTILITY_BAR
#define QTOP_UTILITY_BAR

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/gps.hpp"

#include "UI_TopUtilityBar.h"
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <rover_msgs/msg/battery.hpp>
#include <rover_msgs/msg/detail/wifi_connection__struct.hpp>
#include <QDateTime>
#include <QTimeZone>



class QTopUtilityBar : public QWidget
{
    Q_OBJECT

    static constexpr const char* TOPIC_BATTERY = "/rover/auxiliary/battery";
    static constexpr const char* TOPIC_WIFI_CONNECTION = "/rover/auxiliary/connection_speed";
    static constexpr const size_t DELAY_UPDATE_TIMER_MS = 1000UL;

  public:
  
    QTopUtilityBar(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);

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
    rclcpp::TimerBase::SharedPtr _timer_updateTimer;

    std::shared_ptr<rclcpp::Node> _node;

    Ui::TopUtilityBar _ui;



        
};


#endif //QTOP_UTILITY_BAR