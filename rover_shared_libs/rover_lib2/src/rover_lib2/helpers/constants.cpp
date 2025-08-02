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

    eCamNames getIndexFromName(const std::string& name_)
    {
        size_t index = 0;
        for (const auto& camera : CAMERA_INFO)
        {
            const char* name = camera[static_cast<size_t>(CameraInfo::eInfoType::NAME)];

            if (name == name_)
            {
                return static_cast<eCamNames>(index);
            }
            index++;
        }
        return eCamNames::eLast;
    }
#endif  // defined(__linux__)
}  // namespace Constants::CameraInfo
