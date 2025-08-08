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

#endif  // defined(__linux__)
}  // namespace Constants::CameraInfo
