#include "camera_info.hpp"

namespace CameraInfo
{
    bool getNameFromURL(const std::string& url_, std::string& rName_)
    {
        std::map<std::string, std::string> cameraNameMap;
        for (const auto& [key, value] : CAMERA_URL_MAP)
        {
            cameraNameMap[value] = key;
        }
        
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
}  // namespace CameraInfo
