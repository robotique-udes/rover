#include "constants.hpp"
#include <cstddef>

namespace Constants::CameraInfo
{
#if defined(__linux__)
    bool getNameFromURL(const std::string& url_, std::string& rName_)
    {
        for (const auto& camera : CAMERA_INFO)
        {
            const char* name = camera[static_cast<size_t>(CameraInfo::eInfoType::NAME)];
            const char* url = camera[static_cast<size_t>(CameraInfo::eInfoType::URL)];

            if (url_ == url)
            {
                rName_ = name;
                return true;
            }
        }
        return false;
    }
#endif  // defined(__linux__)
}  // namespace Constants::CameraInfo
