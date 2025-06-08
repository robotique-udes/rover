#include "antenna.hpp"
#include <iostream>
#include <rover_lib2/helpers/constants.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<AntennaNode>());

    rclcpp::shutdown();
    return 0;
}

// Initialize static callback function
size_t AntennaNode::WriteCallback(void* contents, size_t size, size_t nmemb, std::string* s)
{
    size_t newLength = size * nmemb;
    try
    {
        s->append(static_cast<char*>(contents), newLength);
        return newLength;
    }
    catch (std::bad_alloc& e)
    {
        // Handle memory problem
        return 0;
    }
}

AntennaNode::AntennaNode():
    rclcpp::Node("antenna"),
    curl_handle(nullptr),
    is_initialized(false)
{
    // Initialize curl globally - should be called only once in program
    curl_global_init(CURL_GLOBAL_DEFAULT);

    // Create a curl handle that will be reused for all requests
    curl_handle = curl_easy_init();

    if (curl_handle)
    {
        is_initialized = true;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("Antenna"), "Failed to initialize cURL handle");
        is_initialized = false;
    }

    _pub_antenna_status = this->create_publisher<rover_msgs::msg::AntennaStatus>(TOPIC_ANTENNA_STATUS, QOS_DEFAULT);

    _timer_pub = this->create_wall_timer(std::chrono::milliseconds(PUBLISHER_PERIOD_MS),
                                         [this]()
                                         {
                                             rover_msgs::msg::AntennaStatus msg;
                                             // insert function
                                             _pub_antenna_status->publish(msg);
                                         });
}

AntennaNode::~AntennaNode()
{
    // Clean up the curl handle
    if (curl_handle)
    {
        curl_easy_cleanup(curl_handle);
        curl_handle = nullptr;
    }

    // Clean up curl global resources
    curl_global_cleanup();
}

sGetResponse AntennaNode::getHTTPS(const std::string& url_, bool verify_ssl_, std::optional<std::string> cookie_)
{
    sGetResponse response;
    response.success = false;
    response.http_code = 0;
    response.raw_json = "";
    response.error_message = "";

    // Check if service is properly initialized
    if (!is_initialized || !curl_handle)
    {
        response.error_message = "CURLhandle not initialized";
        return response;
    }

    // Clear any previous options
    curl_easy_reset(curl_handle);

    // Buffer for the response
    std::string readBuffer;

    // Set the URL to fetch
    curl_easy_setopt(curl_handle, CURLOPT_URL, url_.c_str());

    // SSL verification settings
    if (!verify_ssl_)
    {
        curl_easy_setopt(curl_handle, CURLOPT_SSL_VERIFYPEER, 0L);
        curl_easy_setopt(curl_handle, CURLOPT_SSL_VERIFYHOST, 0L);
        curl_easy_setopt(curl_handle, CURLOPT_SSL_CIPHER_LIST, "DEFAULT@SECLEVEL=1");
    }

    // Set the callback function to handle the response
    curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, &readBuffer);

    // Set a timeout (in seconds)
    curl_easy_setopt(curl_handle, CURLOPT_TIMEOUT, 10L);

    // Add headers to specify we want JSON
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Accept: application/json");
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl_handle, CURLOPT_HTTPHEADER, headers);

    if (cookie_)
    {
        std::string cookieValue = cookie_.value();
        curl_easy_setopt(curl_handle, CURLOPT_COOKIE, cookieValue.c_str());
    }

    // Execute the request
    CURLcode curlResponseCode = curl_easy_perform(curl_handle);

    // Process results
    if (curlResponseCode == CURLE_OK)
    {
        // Get HTTP response code
        curl_easy_getinfo(curl_handle, CURLINFO_RESPONSE_CODE, &response.http_code);

        response.success = true;
        response.raw_json = readBuffer;
    }
    else
    {
        response.error_message = curl_easy_strerror(curlResponseCode);
    }

    // Clean up headers - must be done after each request
    if (headers)
    {
        curl_slist_free_all(headers);
    }

    return response;
}

std::string AntennaNode::login(const std::string& url)
{
    std::string cookie_value = "";

    // Reset options
    curl_easy_reset(curl_handle);

    // Standard setup
    curl_easy_setopt(curl_handle, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl_handle, CURLOPT_SSL_VERIFYPEER, 0L);
    curl_easy_setopt(curl_handle, CURLOPT_SSL_VERIFYHOST, 0L);
    curl_easy_setopt(curl_handle, CURLOPT_SSL_CIPHER_LIST, "DEFAULT@SECLEVEL=1");

    // Prepare POST data
    std::string postData = "username=" + username + "&password=" + password;
    curl_easy_setopt(curl_handle, CURLOPT_POSTFIELDS, postData.c_str());

    // Setup response buffer
    std::string responseBuffer;
    curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, &responseBuffer);

    // Important: Setup header callback to capture cookies
    std::string headerBuffer;
    curl_easy_setopt(curl_handle, CURLOPT_HEADERFUNCTION, HeaderCallback);
    curl_easy_setopt(curl_handle, CURLOPT_HEADERDATA, &headerBuffer);

    // Execute request
    CURLcode curlCode = curl_easy_perform(curl_handle);

    if (curlCode == CURLE_OK)
    {
        // Extract cookie from headers
        size_t pos = headerBuffer.find("Set-Cookie:");
        if (pos != std::string::npos)
        {
            size_t start = pos + 11;  // Length of "Set-Cookie:"
            size_t end = headerBuffer.find("\r\n", start);
            if (end != std::string::npos)
            {
                cookie_value = headerBuffer.substr(start, end - start);
                // Trim any whitespace
                cookie_value.erase(0, cookie_value.find_first_not_of(" \t"));
            }
        }
    }

    return cookie_value;
}

void AntennaNode::CB_antenna_publisher(void)
{
    rover_msgs::msg::AntennaStatus msg;
    if (Constants::AntennaInfo::ANTENNA_URL_MAP.find("BASE") == Constants::AntennaInfo::ANTENNA_URL_MAP.end())
    {
        msg.success = false;
        msg.status = "Couldn't find the Base antenna URL in the URL map";
        _pub_antenna_status->publish(msg);
        return;
    }

    if (_roverAntennaCookie.empty())
    {
        _roverAntennaCookie = login(Constants::AntennaInfo::ANTENNA_URL_MAP.at("BASE") + "/login.cgi");
    }

    sGetResponse getResponse
        = this->getHTTPS(Constants::AntennaInfo::ANTENNA_URL_MAP.at("BASE") + "/status.cgi", false, _roverAntennaCookie);
    msg.success = getResponse.success;
    msg.status = getResponse.error_message;
    msg.http_code = getResponse.http_code;

    if (getResponse.success)
    {
        // parse
        msg.raw_json = getResponse.raw_json;
    }

    _pub_antenna_status->publish(msg);
}

size_t HeaderCallback(char* buffer, size_t size, size_t nitems, void* userdata)
{
    std::string* headers = static_cast<std::string*>(userdata);
    headers->append(buffer, size * nitems);
    return size * nitems;
}