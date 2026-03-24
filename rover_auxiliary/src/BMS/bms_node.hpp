#ifndef BMS_NODE_HPP
#define BMS_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include "rover_msgs/msg/bms_data.hpp"
#include <utility>
#include <fstream>
#include <iostream>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include "rover_lib2/communication/Serial/serial_com.hpp"

class BMSDataNode : public rclcpp::Node
{
    static constexpr const char* TOPIC_BMS_DATA = "rover/auxiliary/bms_data";
    static constexpr uint64_t DELAY_PUBLISHER_MS = 1000UL;
    static constexpr uint16_t VOLT_DATA_TYPES = 9;
    static constexpr uint16_t AMP_DATA_TYPES = 6;

    enum AmpIndexType
    {
      BATTERY_AMPS,
      AH,
      MAX_AH,
      SOC,
      CHARGE_AMPS,
      LOAD_AMPS
    };

    enum VoltIndexType
    {
      BATTERY,
      LOAD,
      CHARGE,
      CELL_VOLT_START,
      CELL_VOLT_END = 9
    };

  public:
    BMSDataNode();
    void callbackBMSData(void);
    void getData(void);
    void parse(std::string rawOutput, uint16_t dataArray[], uint16_t arraySize);

  private:
    uint16_t _ampArray[AMP_DATA_TYPES];
    uint16_t _voltArray[VOLT_DATA_TYPES];
    rclcpp::Publisher<rover_msgs::msg::BmsData>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer_publisher;
};

#endif