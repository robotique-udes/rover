#ifndef BMS_NODE_HPP
#define BMS_NODE_HPP

#include "rover_msgs/msg/bms_data.hpp"
#include "rover_lib2/communication/Serial/serial_com.hpp"

#include <rover_lib2/helpers/constants.hpp>
#include <rclcpp/rclcpp.hpp>

class BMSDataNode : public rclcpp::Node
{
  private:
    static constexpr const char* TOPIC_BMS_DATA = "/rover/auxiliary/bms_data";
    static constexpr const char* DEVICE_FILE_PATH = "/dev/ttyACM0";
    static constexpr uint64_t DELAY_PUBLISHER_MS = 1000UL;
    static constexpr size_t VOLT_DATA_TYPES = 9;
    static constexpr size_t AMP_DATA_TYPES = 6;

    enum class AmpIndexType : size_t
    {
        BATTERY_AMPS,
        AH,
        MAX_AH,
        SOC,
        CHARGE_AMPS,
        LOAD_AMPS
    };

    enum class VoltIndexType : size_t
    {
        BATTERY,
        LOAD,
        CHARGE,
        CELL_VOLT_START,
        CELL_VOLT_END = 9
    };

  public:
    BMSDataNode();

  private:
    void callbackBMSData(void);
    void getData(void);
    template<size_t N>
    void parse(std::string_view rawOutput_, std::array<uint16_t, N>& dataArray_);

    std::array<uint16_t, AMP_DATA_TYPES> _ampArray;
    std::array<uint16_t, VOLT_DATA_TYPES> _voltArray;
    rclcpp::Publisher<rover_msgs::msg::BmsData>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer_publisher;
    SerialCom _terminal;
};

#endif