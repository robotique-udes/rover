#include "image_capture_glowstick.hpp"


ImageCaptureGlowstick::ImageCaptureGlowstick(std::string cameraURL_) : _cameraURL(cameraURL_)
{
    initCam();
}

ImageCaptureGlowstick::~ImageCaptureGlowstick(void)
{
    _cap.release();
    cv::destroyAllWindows();
}

ImageCaptureGlowstick::initCam(void)
{
    
}