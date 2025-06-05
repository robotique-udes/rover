#include <iostream>
#include <string>
#include <curl/curl.h>
#include <rclcpp/rclcpp.hpp>

// Callback function to handle the response data
static size_t WriteCallback(void *contents, size_t size, size_t nmemb, std::string *s)
{
    size_t newLength = size * nmemb;
    try {
        s->append((char*)contents, newLength);
        return newLength;
    } catch(std::bad_alloc &e) {
        // Handle memory problem
        return 0;
    }
}

int main()
{
    CURL *curl;
    CURLcode res;
    std::string readBuffer;

    // Initialize curl
    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();
    
    if(curl) {
        // Set the URL to fetch
        curl_easy_setopt(curl, CURLOPT_URL, "https://192.168.144.55/status.cgi");
        
        // If you're using HTTPS and want to bypass certificate verification (not recommended for production)
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
        
        // Set the callback function to handle the response
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        
        // Set a timeout (in seconds)
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
        
        // Add headers to specify we want JSON
        struct curl_slist *headers = NULL;
        headers = curl_slist_append(headers, "Accept: application/json");
        headers = curl_slist_append(headers, "Content-Type: application/json");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        
        // Perform the request
        res = curl_easy_perform(curl);
        
        // Check for errors
        if(res != CURLE_OK) {
            std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
        } else {
            // Print the JSON response
            std::cout << "JSON Response:" << std::endl;
            std::cout << readBuffer << std::endl;
            
            // Get HTTP response code
            long http_code = 0;
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
            std::cout << "HTTP Response Code: " << http_code << std::endl;
        }
        
        // Clean up headers
        curl_slist_free_all(headers);
        
        // Clean up curl
        curl_easy_cleanup(curl);
    }
    
    curl_global_cleanup();
    return 0;
}