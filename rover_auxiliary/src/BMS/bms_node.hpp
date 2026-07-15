#ifndef BMS_NODE_HPP
#define BMS_NODE_HPP

#include "rover_msgs/msg/bms_data.hpp"
#include "rover_lib2/communication/Serial/serial_com.hpp"

#include <rover_lib2/helpers/constants.hpp>
#include <rclcpp/rclcpp.hpp>

class BMSDataNode : public rclcpp::Node
{
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
  static constexpr const char* TOPIC_BMS_DATA = "/rover/auxiliary/bms_data";
    static constexpr const char* DEVICE_FILE_PATH = "/dev/ttyACM0";
    static constexpr uint64_t DELAY_PUBLISHER_MS = 1000UL;
    static constexpr size_t VOLT_DATA_TYPES = 13;
    static constexpr size_t AMP_DATA_TYPES = 6;
    static constexpr uint16_t MAX_FAILED_ATTEMPTS = 3;
    static constexpr uint16_t MAX_ECHO_SKIPS = 3;

    void callbackBMSData(void);
    bool getData(void);
    template<size_t N>
    bool parse(std::string_view view_, std::string_view expectedPrefix_, std::array<uint16_t, N>& dataArray_);
    std::optional<std::string> readDataFrame();

    std::array<uint16_t, AMP_DATA_TYPES> _ampArray{};
    std::array<uint16_t, VOLT_DATA_TYPES> _voltArray{};
    rclcpp::Publisher<rover_msgs::msg::BmsData>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer_publisher;
    SerialCom _terminal;
    uint16_t _failedAttempts = 0;
};

#endif