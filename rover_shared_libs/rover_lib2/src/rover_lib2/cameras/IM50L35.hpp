#ifndef IM50L35_HPP
#define IM50L35_HPP

#if defined(__linux__)  // need linux for environment variables

#include "rover_lib2/helpers/log.hpp"

#include <curl/curl.h>

#include <cstdlib>
#include <string>

namespace
{
    inline size_t discardResponse(void* contents, size_t size, size_t nmemb, void* userp)
    {
        (void)contents;
        (void)userp;
        return size * nmemb;
    }
}  // namespace

DEFINE_LOG_NODE(IM50L35, Logger::eNodeState::ON)

namespace IM50L35
{
    enum class IRModes : char
    {
        NIGHT = '2',
        DAY = '3'
    };

    inline void setIR(const std::string& IP_, IRModes IRMode_, bool IREnable_)
    {
        const char* username = std::getenv("CAMERA_USERNAME");
        const char* password = std::getenv("CAMERA_PASSWORD");

        if (!username || !password)
        {
            LOG_ERROR(Logger::Nodes::IM50L35, "CAMERA_USERNAME or CAMERA_PASSWORD environment variable not set");
            return;
        }

        curl_global_init(CURL_GLOBAL_DEFAULT);

        CURL* curl = curl_easy_init();
        if (!curl)
        {
            LOG_ERROR(Logger::Nodes::IM50L35, "Could not intialise CURL");
            curl_global_cleanup();
            return;
        }

        const std::string url = "http://" + IP_ + "/form/IRset";

        const std::string data = std::string("IRmode=") + static_cast<char>(IRMode_) + "&IRenable=" + (IREnable_ ? "1" : "0")
                                 + "&Dualenable=2" + "&luminval=50" + "&IRdelay=2" + "&c2bwthr=20" + "&bw2cthr=70"
                                 + "&PowerMode=0" + "&PowerValue=0";

        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, discardResponse);

        curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_ANY);
        curl_easy_setopt(curl, CURLOPT_USERNAME, username);
        curl_easy_setopt(curl, CURLOPT_PASSWORD, password);

        CURLcode res = curl_easy_perform(curl);

        if (res != CURLE_OK)
        {
            LOG_ERROR(Logger::Nodes::IM50L35, "Curl post error: %s", curl_easy_strerror(res));
        }

        curl_easy_cleanup(curl);
        curl_global_cleanup();
    }
}  // namespace IM50L35

#endif  // defined __linux__
#endif  // IM50L35