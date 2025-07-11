#ifndef ANTENNA_DRIVER_HPP
#define ANTENNA_DRIVER_HPP

#include <string>
#include <functional>
#include <memory>
#include <rover_msgs/msg/antenna_status.hpp>
#include <cpr/cpr.h>
#include <rclcpp/logging.hpp>

class AntennaDriver
{
    static constexpr const char* TOPIC_ANTENNA_STATUS = "/rover/antenna/status";
    static constexpr const char* HTTP_CIPHER = "DEFAULT@SECLEVEL=1";
    static constexpr const char* LOGIN_PAGE = "/login.cgi";
    static constexpr uint8_t MAX_LOGIN_ATTEMPTS = 3U;
    static constexpr int HTTP_SUCCESS_MIN = 200;
    static constexpr int HTTP_SUCCESS_MAX = 300;
    static constexpr int HTTP_UNAUTHORIZED = 401;
    static constexpr int HTTP_FORBIDDEN = 403;

  public:
    AntennaDriver(const std::string& username_, const std::string& password_, rclcpp::logger& logger_);
    void CB_antenna_publisher(rover_msgs::msg::AntennaStatus& msg_);

  private:
    bool login(void);
    bool getIfStats(rover_msgs::msg::AntennaStatus* msg_);
    bool getStatus(rover_msgs::msg::AntennaStatus* msg_);
    void setDebugCB(void);

    rclcpp::Logger& _logger;


    std::shared_ptr<cpr::Session> _session;
    bool _isLoggedIn = false;
    uint8_t _loginAttempts = 0;
    uint64_t _lanRxBytes = 0;
    uint64_t _lanTxBytes = 0;
    uint64_t _wlanRxBytes = 0;
    uint64_t _wlanTxBytes = 0;

    std::string _username;
    std::string _password;
    
};

#endif  // ANTENNA_DRIVER_HPP