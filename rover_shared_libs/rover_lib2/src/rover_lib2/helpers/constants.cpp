#include "constants.hpp"

namespace Constants::CameraInfo
{
#if defined(__linux__)
    bool getNameFromURL(const std::string& url_, std::string& rName_)
    {
        static std::map<std::string, std::string> cameraNameMap = []()
        {
            std::map<std::string, std::string> tempMap;
            for (const auto& [key, value] : Constants::CameraInfo::CAMERA_URL_MAP)
            {
                tempMap[value] = key;
            }
            return tempMap;
        }();

        auto it = cameraNameMap.find(url_);
        if (it != cameraNameMap.end())
        {
            rName_ = it->second;
            return true;
        }
        else
        {
            return false;
        }
    }
#endif  // defined(__linux__)
}  // namespace Constants::CameraInfo
