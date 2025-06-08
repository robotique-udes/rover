#ifndef ANTENNA_NODE_HPP
#define ANTENNA_NODE_HPP

#include <string>
#include <functional>
#include <curl/curl.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/antenna_status.hpp>

static size_t HeaderCallback(char* buffer, size_t size, size_t nitems, void* userdata); //move to class

struct sGetResponse 
{
    bool success;
    long http_code;
    std::string raw_json;
    std::string error_message;
};

class AntennaNode : public rclcpp::Node
{
    static constexpr const char* TOPIC_ANTENNA_STATUS = "/rover/antenna/status";
    static constexpr uint64_t PUBLISHER_PERIOD_MS = 5000UL;

  public:
    // Constructor initializes curl once
    AntennaNode();

    // Destructor cleans up curl
    ~AntennaNode();

    AntennaNode(const AntennaNode&) = delete;
    AntennaNode& operator=(const AntennaNode&) = delete;

    // Response status and data

    // Make a GET request to the specified URL
    sGetResponse getHTTPS(const std::string& url, bool verify_ssl = true, std::optional<std::string> cookie = std::nullopt);

  private:
    rclcpp::Publisher<rover_msgs::msg::AntennaStatus>::SharedPtr _pub_antenna_status;
    rclcpp::TimerBase::SharedPtr _timer_pub;

    void CB_antenna_publisher(void);

    std::string login(const std::string& url);

    // Curl handle that will be reused
    CURL* curl_handle;

    // Initialization state
    bool is_initialized;

    // Callback function for receiving data
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* s); //const?

    std::string _roverAntennaCookie;


    //placeholders
    std::string username = "placeholder";
    std::string password = "placeholder";
};

#endif  // ANTENNA_NODE_HPP