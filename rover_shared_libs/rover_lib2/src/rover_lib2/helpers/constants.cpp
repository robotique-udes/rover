#include "constants.hpp"

namespace Constants::CameraInfo
{
#if defined(__linux__)

    std::optional<std::size_t> getIdFromURL(const std::string& url_)
    {
        std::size_t idCam = 0;
        for (const auto& [key, url] : CAMERA_URL_MAP)
        {
            if (url == url_)
            {
                return idCam;
            }
            ++idCam;
        }
        return std::nullopt;
    }

#endif  // defined(__linux__)
}  // namespace Constants::CameraInfo
