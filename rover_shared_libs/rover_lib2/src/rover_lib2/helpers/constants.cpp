#include "constants.hpp"
#include <cstddef>

namespace Constants::CameraInfo
{
#if defined(__linux__)

    std::optional<eCamNames> getIdFromURL(const std::string& url_)
    {
        for (std::size_t id = 0; id < std::to_underlying(eCamNames::eLast); ++id)
        {
            if (CAMERA_INFO[id][std::to_underlying(eInfoType::URL)] == url_)
            {
                return static_cast<eCamNames>(id);
            }
        }
        return std::nullopt;
    }

    std::string getURLFromId(const std::string& id_)
    {
        for (std::size_t id = 0; id < std::to_underlying(eCamNames::eLast); id++)
        {
            if (CAMERA_INFO[id][std::to_underlying(eInfoType::NAME)] == id_)
            {
                return CAMERA_INFO[id][std::to_underlying(eInfoType::URL)];
            }
        }
        return "";
    }

#endif  // defined(__linux__)
}  // namespace Constants::CameraInfo
