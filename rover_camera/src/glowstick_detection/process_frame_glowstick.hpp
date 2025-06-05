#ifndef PROCESS_FRAME_GLOWSTICK_HPP
#define PROCESS_FRAME_GLOWSTICK_HPP

#include "image_capture_glowstick.hpp"

class ProcessFrame
{
    public:
        ProcessFrame(std::string cameraURL_);

    private:
        ImageCaptureGlowstick _stream;
};


#endif